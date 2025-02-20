//! An example providing a visual demonstration of the 2D cross-platform determinism test
//! in `src/tests/determinism_2d`.
//!
//! This scene is designed to produce a chaotic result engaging:
//!
//! - the contact solver
//! - speculative collision
//! - joints and joint limits
//!
//! Once the simulation has run for a while, a transform hash is computed.
//! The determinism test compares this to the expected value for every PR on multiple platforms using GitHub Actions.
//! Every time simulation behavior changes, the expected hash must be updated.
//!
//! This test is based on the `FallingHinges` test in the Box2D physics engine:
//! <https://github.com/erincatto/box2d/blob/90c2781f64775085035655661d5fe6542bf0fbd5/samples/sample_determinism.cpp>

use avian3d::{
    math::{AdjustPrecision, Scalar, Vector, PI},
    prelude::*,
};
use bevy::tasks::futures_lite::StreamExt;
use bevy::{
    color::palettes::tailwind::CYAN_400, input::common_conditions::input_just_pressed, prelude::*,
    prelude::*,
};
use bytemuck::{Pod, Zeroable};

// How many steps to record the hash for.
const STEP_COUNT: usize = 500;

const ROWS: u32 = 30;
const COLUMNS: u32 = 4;

use rand::seq::SliceRandom;
use rand::thread_rng;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            PhysicsPlugins::default().with_length_unit(0.5),
            PhysicsDebugPlugin::default(),
        ))
        .init_resource::<Step>()
        .add_systems(Startup, (setup_scene, setup_ui))
        .add_systems(PostProcessCollisions, ignore_joint_collisions)
        .add_systems(FixedUpdate, update_hash)
        .add_systems(
            PreUpdate,
            // Reset the scene when the R key is pressed.
            (clear_scene, setup_scene)
                .chain()
                .run_if(input_just_pressed(KeyCode::KeyR)),
        )
        .run();
}

#[derive(Resource, Default, Deref, DerefMut)]
struct Step(usize);

fn setup_scene(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    // Directional light
    commands.spawn((
        DirectionalLight {
            illuminance: 5000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::default().looking_at(Vec3::new(-1.0, -2.5, -1.5), Vec3::Y),
    ));

    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_translation(Vec3::new(0.0, 12.0, 40.0)).looking_at(Vec3::Y * 5.0, Vec3::Y),
    ));

    let cube_mesh = meshes.add(Cuboid::default());

    // Ground
    commands.spawn((
        Mesh3d(cube_mesh.clone()),
        MeshMaterial3d(materials.add(Color::srgb(0.7, 0.7, 0.8))),
        Transform::from_xyz(0.0, -2.0, 0.0).with_scale(Vec3::new(100.0, 1.0, 100.0)),
        RigidBody::Static,
        Collider::cuboid(1.0, 1.0, 1.0),
    ));

    let half_size = 0.5;

    let offset = 0.4 * half_size;
    let delta_x = 10.0 * half_size;
    let x_root = -0.5 * delta_x * (COLUMNS as f32 - 1.0);

    let mut cols = (0..COLUMNS).collect::<Vec<u32>>();
    cols.shuffle(&mut rand::thread_rng());

    for col in cols {
        let x = x_root + col as f32 * delta_x;

        // let mut prev_entity = None;

        for row in 0..ROWS {
            commands.spawn((
                Name::new("Square ({col}, {row})"),
                RigidBody::Dynamic,
                Mesh3d(cube_mesh.clone()),
                MeshMaterial3d(materials.add(Color::srgb(0.2, 0.7, 0.9))),
                Transform::from_xyz(
                    x + offset * row as f32,
                    half_size + 2.0 * half_size * row as f32,
                    0.0,
                ),
                Collider::cuboid(1.0, 1.0, 1.0),
            ));
        }
    }
}

#[derive(Component)]
struct StepText;

#[derive(Component)]
struct HashText;

fn setup_ui(mut commands: Commands) {
    let font = TextFont {
        font_size: 20.0,
        ..default()
    };

    commands
        .spawn((
            Text::new("Step: "),
            font.clone(),
            Node {
                position_type: PositionType::Absolute,
                top: Val::Px(5.0),
                left: Val::Px(5.0),
                ..default()
            },
        ))
        .with_child((TextSpan::new("0"), font.clone(), StepText));

    commands
        .spawn((
            Text::new("Hash: "),
            font.clone(),
            Node {
                position_type: PositionType::Absolute,
                top: Val::Px(30.0),
                left: Val::Px(5.0),
                ..default()
            },
        ))
        .with_child((TextSpan::default(), font.clone(), HashText));

    commands.spawn((
        Text::new("Press R to reset scene"),
        font.clone(),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(5.0),
            right: Val::Px(5.0),
            ..default()
        },
    ));
}

// TODO: This should be an optimized built-in feature for joints.
fn ignore_joint_collisions(joints: Query<&RevoluteJoint>, mut collisions: ResMut<Collisions>) {
    for joint in &joints {
        collisions.remove_collision_pair(joint.entity1, joint.entity2);
    }
}

fn clear_scene(
    mut commands: Commands,
    query: Query<
        Entity,
        Or<(
            With<RigidBody>,
            With<Collider>,
            With<RevoluteJoint>,
            With<Camera>,
        )>,
    >,
    mut step: ResMut<Step>,
) {
    step.0 = 0;
    for entity in &query {
        commands.entity(entity).despawn_recursive();
    }
}

#[derive(Pod, Zeroable, Clone, Copy)]
#[repr(C)]
struct Isometry {
    translation: Vector,
    rotation: [f32; 4],
}

fn update_hash(
    transforms: Query<(&Position, &Rotation), With<RigidBody>>,
    mut step_text: Single<&mut TextSpan, With<StepText>>,
    mut hash_text: Single<&mut TextSpan, (With<HashText>, Without<StepText>)>,
    mut step: ResMut<Step>,
) {
    step_text.0 = step.to_string();
    step.0 += 1;

    if step.0 > STEP_COUNT {
        return;
    }

    let mut hash = 5381;

    let mut transforms_vec: Vec<_> = transforms.iter().collect(); // Collect into a Vec first
    transforms_vec.sort_by(|a, b| a.0.x.partial_cmp(&b.0.x).expect("Comparison failed"));

    for (position, rotation) in transforms_vec {
        let isometry = Isometry {
            translation: position.0.into(),
            rotation: rotation.0.into(),
        };
        hash = djb2_hash(hash, bytemuck::bytes_of(&isometry));
    }

    if step.0 == STEP_COUNT {
        hash_text.0 = format!("0x{:x} (step {})", hash, step.0);
    } else {
        hash_text.0 = format!("0x{:x}", hash);
    }
}

fn djb2_hash(mut hash: u32, data: &[u8]) -> u32 {
    for &byte in data {
        hash = (hash << 5).wrapping_add(hash).wrapping_add(byte as u32);
    }
    hash
}
