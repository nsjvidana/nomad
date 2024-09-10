use bevy::{app::{App, Startup}, math::Vec3, prelude::{default, Camera3dBundle, Commands, Transform, TransformBundle}, DefaultPlugins};
use bevy_flycam::{FlyCam, MovementSettings, NoCameraPlayerPlugin};
use bevy_rapier3d::{na::coordinates::X, plugin::RapierPhysicsPlugin, prelude::{Collider, CollisionGroups, FixedJoint, GenericJoint, GenericJointBuilder, Group, ImpulseJoint, JointAxesMask, JointAxis, MultibodyJoint, RevoluteJointBuilder, RigidBody, SphericalJointBuilder}, render::RapierDebugRenderPlugin};

fn main() {
    let mut app = App::new();
    app.add_plugins((
        DefaultPlugins,
        RapierPhysicsPlugin::<()>::default(),
        RapierDebugRenderPlugin::default(),
        NoCameraPlayerPlugin
    ));

    app
        .add_systems(Startup, startup);

    //flycam config
    app.world_mut().resource_mut::<MovementSettings>()
        .speed = 2.5;

    app.run();
}

fn startup(
    mut commands: Commands
) {
    //camera
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(-4., 8., 4.)
                .looking_at(Vec3::Y * 5., Vec3::Y),
            ..default()
        },
        FlyCam,
        Collider::ball(0.1)
    ));
    //ground
    commands.spawn((
        Transform::from_xyz(0., -5., 0.),
        RigidBody::Fixed,
        Collider::cuboid(10., 0.1, 10.),
    ));


    {//player
        let radius = 0.1;
        let arm_upper_len = 2.;
        let arm_lower_len = 2.5;
        let torso_len = 4.;
        let leg_upper_len = 3.;
        let leg_lower_len = 3.1;

        macro_rules! make_part {
            ($length:expr) => {
                commands.spawn((
                    TransformBundle::from_transform(Transform::from_xyz(0., 5., 0.)),
                    RigidBody::Dynamic,
                    Collider::capsule_y(($length/2.) - (radius), radius),
                ))
            };
        }

        let torso = commands.spawn((
            TransformBundle::from_transform(Transform::from_xyz(0., 5., 0.)),
            RigidBody::Fixed,
            Collider::capsule_y((torso_len/2.) - (radius), radius),
            CollisionGroups {
                memberships: Group::GROUP_1,
                filters: Group::GROUP_2,
            }
        )).id();

        
        // .limits(JointAxis::AngX, [-150f32.to_radians(), 60f32.to_radians()])
        // .limits(JointAxis::AngY, [-90f32.to_radians() , 90f32.to_radians()])
        // .limits(JointAxis::AngZ, [0.                  , 180f32.to_radians()])

        let motor_vel = 90f32.to_radians();

        let shld_x_j = RevoluteJointBuilder::new(Vec3::X)
            .local_anchor1(Vec3::Y * (torso_len/2.))
            .limits([-150f32.to_radians(), 60f32.to_radians()])
            .motor(0., motor_vel, 1., 0.0)
            .motor_max_force(20.);
        let shld_y_j = RevoluteJointBuilder::new(Vec3::Y)
            .limits([-90f32.to_radians(), 90f32.to_radians()])
            .motor(0., motor_vel, 1., 0.0)
            .motor_max_force(20.);
        let shld_z_j = RevoluteJointBuilder::new(Vec3::Z)
            .local_anchor2(Vec3::Y * (arm_upper_len/2.))
            .limits([0., 180f32.to_radians()])
            .motor(90f32.to_radians(), motor_vel, 1., 0.0)
            .motor_max_force(20.);
        let mut no_collision_j = FixedJoint::default();
            no_collision_j.data.raw = default();
            no_collision_j.set_contacts_enabled(false);


        let shld_x = commands.spawn((RigidBody::Dynamic, MultibodyJoint::new(torso, shld_x_j.into()))).id();
        let shld_y = commands.spawn((RigidBody::Dynamic, MultibodyJoint::new(shld_x, shld_y_j.into()))).id();
        let arm_r_upper = make_part!(arm_upper_len)
            .insert(MultibodyJoint::new(shld_y, shld_z_j.into()))
            .insert(ImpulseJoint::new(torso, no_collision_j))
            .id();

    }
}
