#![warn(clippy::all, clippy::pedantic)]

use macroquad::{
    color::Color,
    input::{is_key_released, KeyCode},
    miniquad::date,
    rand::{self as macroquad_rand, srand},
    shapes::draw_circle,
    window::{clear_background, next_frame, Conf},
};
use rand::{rngs::StdRng, Rng, SeedableRng};
use rand_distr::Standard;
use rapier2d::{
    dynamics::{
        CCDSolver, ImpulseJointSet, IntegrationParameters, IslandManager, MultibodyJointSet,
        RigidBodyBuilder, RigidBodyHandle, RigidBodySet,
    },
    geometry::{
        BroadPhase, ColliderBuilder, ColliderSet, CollisionEvent, CollisionEventFlags, NarrowPhase,
    },
    math::Isometry,
    na::{vector, DVector, Vector2},
    pipeline::{ActiveEvents, ChannelEventCollector, PhysicsPipeline, QueryPipeline},
    prelude::nalgebra,
};

pub const BLUE_CRAYOLA: Color = Color {
    r: 33.0 / 255.0,
    g: 118.0 / 255.0,
    b: 1.0,
    a: 1.0,
};

pub const CARROT_ORANGE: Color = Color {
    r: 247.0 / 255.0,
    g: 152.0 / 255.0,
    b: 36.0 / 255.0,
    a: 1.0,
};
pub const CELESTIAL_BLUE: Color = Color {
    r: 51.0 / 255.0,
    g: 161.0 / 255.0,
    b: 253.0,
    a: 1.0,
};
pub const GUNMETAL: Color = Color {
    r: 49.0 / 255.0,
    g: 57.0 / 255.0,
    b: 60.0 / 255.0,
    a: 1.0,
};
pub const SUNGLOW: Color = Color {
    r: 4253.0 / 255.0,
    g: 202.0 / 255.0,
    b: 64.0 / 255.0,
    a: 1.0,
};

const BALL_COLOURS: [Color; 4] = [BLUE_CRAYOLA, CARROT_ORANGE, CELESTIAL_BLUE, SUNGLOW];

const WINDOW_WIDTH: f32 = 1366.0;
const WINDOW_HEIGHT: f32 = 768.0;
// 1 metre is 50 pixels
const PHYSICS_SCALE: f32 = 50.0;
const BALL_RADIUS: f32 = 0.6;

#[derive(Debug)]
struct Ball {
    radius: f32,
    position: Vector2<f32>,
    physics_handle: Option<RigidBodyHandle>,
    colour: Color,
}

impl Default for Ball {
    fn default() -> Ball {
        Ball {
            radius: BALL_RADIUS,
            position: vector![
                WINDOW_WIDTH / (2.0 * PHYSICS_SCALE),
                (2.0 * BALL_RADIUS) - (1.0 * WINDOW_HEIGHT / PHYSICS_SCALE)
            ],
            physics_handle: None,
            colour: BALL_COLOURS[macroquad_rand::gen_range(0, BALL_COLOURS.len())],
        }
    }
}

impl Ball {
    fn physics_handle(&mut self, physics_handle: RigidBodyHandle) -> &mut Ball {
        self.physics_handle = Some(physics_handle);
        self
    }
}

fn conf() -> Conf {
    #[allow(clippy::cast_possible_truncation)]
    Conf {
        window_title: String::from("Macroquad Rapier Bubbles"),
        window_width: WINDOW_WIDTH as i32,
        window_height: WINDOW_HEIGHT as i32,
        high_dpi: true,
        ..Default::default()
    }
}

fn create_physics_for_ball(
    ball: &Ball,
    rigid_body_set: &mut RigidBodySet,
    collider_set: &mut ColliderSet,
    normal_distribution: &mut StdRng,
) -> RigidBodyHandle {
    // Standard generates values in the [0,1) range
    let pseudo_random_value: f32 = normal_distribution.sample(Standard);
    let x_velocity: f32 = (2.0 * pseudo_random_value) - 1.0;
    let linear_velocity = vector![-1.0 * x_velocity, 1.0];
    let rigid_body = RigidBodyBuilder::dynamic()
        .translation(ball.position)
        .linvel(linear_velocity)
        .build();
    let collider = ColliderBuilder::ball(BALL_RADIUS)
        .restitution(0.0)
        .density(0.001)
        .active_events(ActiveEvents::COLLISION_EVENTS)
        .build();
    let ball_body_handle = rigid_body_set.insert(rigid_body);
    collider_set.insert_with_parent(collider, ball_body_handle, rigid_body_set);
    ball_body_handle
}

fn create_ceiling(ceiling_width: f32, max_balls: u32, collider_set: &mut ColliderSet) {
    let collider_half_thickness = 0.05;
    let nsubdivs: usize = (max_balls * 2)
        .try_into()
        .expect("Expected fewer subdivisions");
    let heights = DVector::from_fn(nsubdivs + 1, |i, _| if i % 2 == 0 { -1.2 } else { 0.0 });
    let collider =
        ColliderBuilder::heightfield(heights, vector![ceiling_width, collider_half_thickness])
            .translation(vector![
                0.5 * WINDOW_WIDTH / PHYSICS_SCALE,
                -1.0 * collider_half_thickness
            ])
            .friction(1.0)
            .restitution(0.0)
            .build();
    collider_set.insert(collider);
}

fn create_ground(collider_set: &mut ColliderSet) {
    let collider_thickness = 0.1;
    let collider = ColliderBuilder::cuboid(100.0, collider_thickness)
        .translation(vector![
            0.0,
            (WINDOW_HEIGHT / -PHYSICS_SCALE) - 0.5 * collider_thickness
        ])
        .sensor(true)
        .build();
    collider_set.insert(collider);
}

fn create_side_walls(gap: f32, collider_set: &mut ColliderSet) {
    // left wall
    let collider_half_thickness = 0.05;
    let collider =
        ColliderBuilder::cuboid(0.5 * WINDOW_HEIGHT / PHYSICS_SCALE, collider_half_thickness)
            .position(Isometry::new(
                vector![
                    gap - collider_half_thickness,
                    (WINDOW_HEIGHT / (-2.0 * PHYSICS_SCALE))
                ],
                std::f32::consts::FRAC_PI_2,
            ))
            .build();
    collider_set.insert(collider);

    // right wall
    let collider_half_thickness = 0.05;
    let collider =
        ColliderBuilder::cuboid(0.5 * WINDOW_HEIGHT / PHYSICS_SCALE, collider_half_thickness)
            .position(Isometry::new(
                vector![
                    (WINDOW_WIDTH / PHYSICS_SCALE) + collider_half_thickness - gap,
                    (WINDOW_HEIGHT / (-2.0 * PHYSICS_SCALE))
                ],
                3.0 * std::f32::consts::FRAC_PI_2,
            ))
            .build();
    collider_set.insert(collider);
}

fn draw_balls(balls: &[Ball]) {
    for ball in balls {
        let Ball {
            colour,
            position,
            radius,
            ..
        } = ball;
        draw_circle(
            PHYSICS_SCALE * position.x,
            -PHYSICS_SCALE * position.y,
            PHYSICS_SCALE * radius,
            *colour,
        );
    }
}

fn update_balls(balls: &mut [Ball], rigid_body_set: &RigidBodySet) {
    for ball in balls {
        if let Some(handle) = ball.physics_handle {
            let ball_body = &rigid_body_set[handle];
            ball.position = *ball_body.translation();
        }
    }
}

#[macroquad::main(conf)]
async fn main() {
    // seed macroquad random number generator
    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    {
        srand(date::now().floor() as u64);
    }

    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();

    // Create the ground
    create_ground(&mut collider_set);

    // Create the ceiling
    // maximum number of balls that can fit across the window
    let max_balls: u32;
    #[allow(
        clippy::cast_precision_loss,
        clippy::cast_sign_loss,
        clippy::cast_possible_truncation
    )]
    {
        max_balls = ((WINDOW_WIDTH / PHYSICS_SCALE) / (2.0 * BALL_RADIUS)).floor() as u32;
    }

    let ceiling_width: f32;
    #[allow(clippy::cast_precision_loss)]
    {
        ceiling_width = max_balls as f32 * BALL_RADIUS * 2.0;
    }
    create_ceiling(ceiling_width, max_balls, &mut collider_set);

    // gap at left and right edge of window
    let gap = 0.5 * ((WINDOW_WIDTH / PHYSICS_SCALE) - ceiling_width);

    // Create the left and right wall
    create_side_walls(gap, &mut collider_set);

    // Create ball
    let mut normal_distribution = StdRng::from_entropy();
    let mut new_ball = Ball::default();
    let ball_body_handle = create_physics_for_ball(
        &new_ball,
        &mut rigid_body_set,
        &mut collider_set,
        &mut normal_distribution,
    );
    new_ball.physics_handle(ball_body_handle);
    let mut balls: Vec<Ball> = Vec::with_capacity(256);
    balls.push(new_ball);

    // Create other structures necessary for the simulation
    // positive gravity indicates it is applied upwards (reversed)
    let gravity = vector![0.0, 1.0];
    let integration_parameters = IntegrationParameters::default();
    let mut physics_pipeline = PhysicsPipeline::new();
    let mut island_manager = IslandManager::new();
    let mut broad_phase = BroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut impulse_joint_set = ImpulseJointSet::new();
    let mut multibody_joint_set = MultibodyJointSet::new();
    let mut ccd_solver = CCDSolver::new();
    let mut query_pipeline = QueryPipeline::new();
    let physics_hooks = ();
    let (collision_send, collision_recv) = crossbeam::channel::unbounded();
    let (contact_force_send, _contact_force_recv) = crossbeam::channel::unbounded();
    let event_handler = ChannelEventCollector::new(collision_send, contact_force_send);

    let mut paused = false;

    // run the game loop, stepping the simulation once per frame.
    loop {
        if is_key_released(KeyCode::Escape) {
            break;
        }

        clear_background(GUNMETAL);
        draw_balls(&balls);

        if !paused {
            physics_pipeline.step(
                &gravity,
                &integration_parameters,
                &mut island_manager,
                &mut broad_phase,
                &mut narrow_phase,
                &mut rigid_body_set,
                &mut collider_set,
                &mut impulse_joint_set,
                &mut multibody_joint_set,
                &mut ccd_solver,
                Some(&mut query_pipeline),
                &physics_hooks,
                &event_handler,
            );

            // update ball positions (used for drawing)
            update_balls(&mut balls, &rigid_body_set);

            // wait for existing balls to settle before spawning a new one
            if island_manager.active_dynamic_bodies().is_empty() {
                let mut new_ball = Ball::default();
                let ball_body_handle = create_physics_for_ball(
                    &new_ball,
                    &mut rigid_body_set,
                    &mut collider_set,
                    &mut normal_distribution,
                );
                new_ball.physics_handle(ball_body_handle);
                balls.push(new_ball);
            }

            // end simulation if a ball touches the ground
            while let Ok(collision_event) = collision_recv.try_recv() {
                if let CollisionEvent::Started(
                    _collider_handle_1,
                    _collider_handle_2,
                    CollisionEventFlags::SENSOR,
                ) = collision_event
                {
                    paused = true;
                };
            }
        }
        next_frame().await;
    }
}
