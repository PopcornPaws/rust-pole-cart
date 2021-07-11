extern crate nalgebra as na;

use na::{Vector2, Point2};
use ncollide2d::pipeline::CollisionGroups;
use ncollide2d::shape::{Cuboid, ShapeHandle};
use nphysics2d::force_generator::{DefaultForceGeneratorSet, ForceGenerator};
use nphysics2d::joint::DefaultJointConstraintSet;
use nphysics2d::joint::{PrismaticConstraint, RevoluteConstraint};
use nphysics2d::math::{Force, ForceType, Velocity};
use nphysics2d::object::{Body, BodyPartHandle, BodyStatus, ColliderDesc, DefaultBodySet, DefaultColliderSet, Ground, RigidBodyDesc};
use nphysics2d::solver::IntegrationParameters;
use nphysics2d::world::{DefaultMechanicalWorld, DefaultGeometricalWorld};

use nphysics_testbed2d::Testbed;

pub fn init_world(testbed: &mut Testbed) { 
    // World
    let mut mechanical_world = DefaultMechanicalWorld::new(Vector2::new(0.0, -9.81));
    let mut geometrical_world = DefaultGeometricalWorld::new();
    let mut bodies = DefaultBodySet::new();
    let mut colliders = DefaultColliderSet::new();
    let mut joint_constraints = DefaultJointConstraintSet::new();
    let force_generators = DefaultForceGeneratorSet::new();
    // Collision group for the cart and the pole
    const CART_GROUP_ID: usize = 0;
    const POLE_GROUP_ID: usize = 1;
    let mut cart_group = CollisionGroups::new();
    cart_group.set_membership(&[CART_GROUP_ID]);
    cart_group.set_whitelist(&[CART_GROUP_ID]);
    let mut pole_group = CollisionGroups::new();
    pole_group.set_membership(&[POLE_GROUP_ID]);
    pole_group.set_whitelist(&[POLE_GROUP_ID]);
    // Ground
    let ground_size = 25.0;
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector2::new(ground_size, 1.0)));

    let ground_handle = bodies.insert(Ground::new());
    let co = ColliderDesc::new(ground_shape)
        .collision_groups(cart_group)
        .translation(Vector2::y() * -2.0)
        .build(BodyPartHandle(ground_handle, 0));

    colliders.insert(co);

    let geometry = ShapeHandle::new(Cuboid::new(Vector2::new(1.0, 0.5)));
    let collider_desc = ColliderDesc::new(geometry).density(1.0);

    let mut parent = BodyPartHandle(ground_handle, 0);
    let first_anchor = Point2::new(0.0, 1.0);
    let other_anchor = Point2::new(0.0, 2.0);
    let mut translation = first_anchor.coords;

    for j in 0usize..3 {
        let rb = RigidBodyDesc::new().translation(translation).build();
        let rb_handle = bodies.insert(rb);
        let co = collider_desc.build(BodyPartHandle(rb_handle, 0));
        colliders.insert(co);

        let mut prismatic_constraint = PrismaticConstraint::new(
            parent,
            BodyPartHandle(rb_handle, 0),
            if j == 0 { first_anchor } else { other_anchor },
            Vector2::x_axis(),
            Point2::origin(),
        );

        //prismatic_constraint.enable_min_offset(-rad * 0.2);
        joint_constraints.insert(prismatic_constraint);

        translation += other_anchor.coords;
        parent = BodyPartHandle(rb_handle, 0);
    }

    //testbed.add_callback(move |_, _, bodies, _, _, _| {
    //    let force = Force::linear(Vector2::new(0.0, 10.0));
    //});

    // Run the simulation.
    testbed.set_ground_handle(Some(ground_handle));
    testbed.set_world(
        mechanical_world,
        geometrical_world,
        bodies,
        colliders,
        joint_constraints,
        force_generators,
    );
    testbed.look_at(Point2::new(0.0, 0.0), 50.0);
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Kinematic body", init_world)]);
    testbed.run();
}
