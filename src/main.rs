use bevy::{app::{App, PostUpdate, Startup}, math::Vec3, prelude::{default, Added, Camera3dBundle, Commands, Component, Entity, IntoSystemConfigs, Parent, Query, ResMut, Transform, TransformBundle}, DefaultPlugins};
use bevy_flycam::{FlyCam, MovementSettings, NoCameraPlayerPlugin};
use bevy_rapier3d::{plugin::{systems::init_joints, RapierContext, RapierPhysicsPlugin}, prelude::{Collider, CollisionGroups, FixedJoint, GenericJoint, GenericJointBuilder, Group, ImpulseJoint, JointAxesMask, JointAxis, MultibodyJoint, RapierMultibodyJointHandle, RevoluteJointBuilder, RigidBody, SphericalJointBuilder, TypedJoint}, render::RapierDebugRenderPlugin};

fn main() {
    let mut app = App::new();
    app.add_plugins((
        DefaultPlugins,
        RapierPhysicsPlugin::<()>::default(),
        RapierDebugRenderPlugin::default(),
        NoCameraPlayerPlugin
    ));

    app
        .add_systems(Startup, startup)
        .add_systems(PostUpdate, chain_arm_joints.after(init_joints));

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

    
    let motor_vel = 90f32.to_radians();
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


    {//testing
        let seg_len = 0.5;
        let root = make_part!(seg_len)
            .insert(RigidBody::Fixed)
            .id();
        let mut prev = root;
        let joint = *SphericalJointBuilder::new()
            .local_anchor1(Vec3::Y * seg_len/2.)
            .local_anchor2(Vec3::Y * -seg_len/2.)
            .motor(JointAxis::AngX, 0., motor_vel, 1., 0.)
            .motor(JointAxis::AngY, 0., motor_vel, 1., 0.)
            .motor(JointAxis::AngZ, 0., motor_vel, 1., 0.)
            .build()
            .set_contacts_enabled(false);
        let mut branchoff_seg = None;
        for i in 0..5 {
            let segment = make_part!(seg_len)
                .insert(MultibodyJoint::new(prev, joint.into()))
                .id();
            prev = segment;

            if i == 1 {
                branchoff_seg = Some(segment);
            }
        }

        let branchoff_seg = branchoff_seg.unwrap();
        let shld_x_j = RevoluteJointBuilder::new(Vec3::X)
            .local_anchor1(Vec3::Y * (torso_len/2.))
            .limits([-150f32.to_radians(), 60f32.to_radians()])
            .motor(0., motor_vel, 1., 0.0);
        //todo: create a branch multibody separately and connect its root to the main multibody
    }

    return;

    {//player

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


        let shld_x_j = RevoluteJointBuilder::new(Vec3::X)
            .local_anchor1(Vec3::Y * (torso_len/2.))
            .limits([-150f32.to_radians(), 60f32.to_radians()])
            .motor(0., motor_vel, 1., 0.0);
        let shld_y_j = RevoluteJointBuilder::new(Vec3::Y)
            .limits([-90f32.to_radians(), 90f32.to_radians()])
            .motor(0., motor_vel, 1., 0.0);
        let shld_z_j = RevoluteJointBuilder::new(Vec3::Z)
            .local_anchor2(Vec3::Y * (arm_upper_len/2.))
            .limits([0., 180f32.to_radians()])
            .motor(90f32.to_radians(), motor_vel, 1., 0.0);
        let mut no_collision_j = FixedJoint::default();
            no_collision_j.data.raw = default();
            no_collision_j.set_contacts_enabled(false);


        let shld_x = commands.spawn(RigidBody::Dynamic).id();
        let shld_y = commands.spawn(RigidBody::Dynamic).id();
        let arm_r_upper = make_part!(arm_upper_len)
            .insert(ImpulseJoint::new(torso, no_collision_j))
            .id();

        let elb_x_j = RevoluteJointBuilder::new(Vec3::X)
            .local_anchor1(Vec3::Y * (-arm_upper_len/2.))
            .limits([0., 45f32.to_radians()])
            .motor(0., motor_vel, 1., 0.0);
        let elb_y_j = RevoluteJointBuilder::new(Vec3::Y)
            .local_anchor2(Vec3::Y * (arm_lower_len/2.))
            .limits([0., 180f32.to_radians()])
            .motor(0., motor_vel, 1., 0.0);
        let elb_x = commands.spawn(RigidBody::Dynamic).id();
        let arm_r_lower = make_part!(arm_lower_len)
            .insert(ImpulseJoint::new(arm_r_upper, no_collision_j))
            .id();
        
        commands.spawn(Arm {
            chain: vec![
                (torso, None),
                (shld_x, Some(shld_x_j.build().data)),
                (shld_y, Some(shld_y_j.build().data)),
                (arm_r_upper, Some(shld_z_j.build().data)),
                (elb_x, Some(elb_x_j.build().data)),
                (arm_r_lower, Some(elb_y_j.build().data))
            ]
        });

    }
}

#[derive(Component)]
pub struct Arm {
    pub chain: Vec<(Entity, Option<GenericJoint>)>
}

fn chain_arm_joints(
    mut commands: Commands,
    mut rapier_ctx: ResMut<RapierContext>,
    new_arm_q: Query<&Arm, Added<Arm>>
) {
    for arm in new_arm_q.iter() {
        for (i, (body_ent, joint_opt)) in arm.chain.iter().enumerate() {
            if i == 0 { continue; }
            let parent_ent = arm.chain.get(i-1).map(|val| val.0);
            if parent_ent.is_none() || joint_opt.is_none() { continue; }
            
            let body1_opt = rapier_ctx.entity2body().get(&parent_ent.unwrap());
            let body2_opt = rapier_ctx.entity2body().get(body_ent);

            if let (Some(body1), Some(body2)) = (body1_opt, body2_opt) {
                let body1 = *body1;
                let body2 = *body2;
                let body_ent = *body_ent;
                if let Some(mb_handle) = rapier_ctx.multibody_joints.insert(
                    body1,
                    body2, 
                    joint_opt.unwrap().into_rapier(), 
                    true
                ) {
                    commands.entity(body_ent)
                        .insert(RapierMultibodyJointHandle(mb_handle));
                } else {
                    panic!("Failed to create multibody arm: loop detected.")
                }
            }
        }
    }
}
