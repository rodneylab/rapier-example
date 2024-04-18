use macroquad::{
    color::Color,
    input::{is_key_released, KeyCode},
    shapes::draw_circle,
    window::{clear_background, next_frame, Conf},
};
use rapier2d::{
    dynamics::{
        CCDSolver, ImpulseJointSet, IntegrationParameters, IslandManager, MultibodyJointSet,
        RigidBodyBuilder, RigidBodySet,
    },
    geometry::{BroadPhase, ColliderBuilder, ColliderSet, NarrowPhase},
    na::{vector, Vector2},
    pipeline::{PhysicsPipeline, QueryPipeline},
    prelude::nalgebra,
};

pub const CARROT_ORANGE: Color = Color {
    r: 247.0 / 255.0,
    g: 152.0 / 255.0,
    b: 36.0 / 255.0,
    a: 1.0,
};
pub const GUNMETAL: Color = Color {
    r: 49.0 / 255.0,
    g: 57.0 / 255.0,
    b: 60.0 / 255.0,
    a: 1.0,
};

const WINDOW_WIDTH: f32 = 1366.0;
const WINDOW_HEIGHT: f32 = 768.0;
// 1 metre is 50 pixels
const PHYSICS_SCALE: f32 = 50.0;
const BALL_RADIUS: f32 = 0.6;

#[derive(Debug)]
struct Ball {
    radius: f32,
    position: Vector2<f32>,
}

impl Default for Ball {
    fn default() -> Ball {
        Ball {
            radius: BALL_RADIUS,
            position: vector![
                WINDOW_WIDTH / (2.0 * PHYSICS_SCALE),
                -WINDOW_HEIGHT / (2.0 * PHYSICS_SCALE),
            ],
        }
    }
}

fn conf() -> Conf {
    #[allow(clippy::cast_possible_truncation)]
    Conf {
        window_title: String::from("Rapier Macroquad Hello World"),
        window_width: WINDOW_WIDTH as i32,
        window_height: WINDOW_HEIGHT as i32,
        high_dpi: true,
        ..Default::default()
    }
}

#[macroquad::main(conf)]
async fn main() {
    let mut ball = Ball::default();

    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();

    // Create the ground
    let collider_half_thickness = 0.05;
    let collider = ColliderBuilder::cuboid(
        WINDOW_WIDTH / (2.0 * PHYSICS_SCALE),
        collider_half_thickness,
    )
    .translation(vector![
        0.0,
        (WINDOW_HEIGHT / -PHYSICS_SCALE) - collider_half_thickness
    ])
    .build();
    collider_set.insert(collider);

    // Create the bouncing ball
    let rigid_body = RigidBodyBuilder::dynamic()
        .translation(ball.position)
        .build();
    let collider = ColliderBuilder::ball(BALL_RADIUS).restitution(0.7).build();
    let ball_body_handle = rigid_body_set.insert(rigid_body);
    collider_set.insert_with_parent(collider, ball_body_handle, &mut rigid_body_set);

    // Create other structures necessary for the simulation
    let gravity = vector![0.0, -9.81];
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
    let event_handler = ();

    // run the game loop, stepping the simulation once per frame.
    loop {
        clear_background(GUNMETAL);

        if is_key_released(KeyCode::Escape) {
            break;
        }

        draw_circle(
            PHYSICS_SCALE * ball.position.x,
            -PHYSICS_SCALE * ball.position.y,
            PHYSICS_SCALE * ball.radius,
            CARROT_ORANGE,
        );

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

        let ball_body = &rigid_body_set[ball_body_handle];
        println!("Ball altitude: {}", ball_body.translation().y);

        // update ball position (used for drawing)
        ball.position = *ball_body.translation();

        next_frame().await;
    }
}
