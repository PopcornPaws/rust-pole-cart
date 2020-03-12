extern crate nalgebra as na;

use na::{Vector2, Point2};
use ncollide2d::pipeline::CollisionGroups;
use ncollide2d::shape::{Cuboid, ShapeHandle};
use nphysics2d::force_generator::{DefaultForceGeneratorSet};
use nphysics2d::joint::DefaultJointConstraintSet;
use nphysics2d::joint::{PrismaticConstraint, RevoluteConstraint};
use nphysics2d::math::{Force, ForceType};
use nphysics2d::object::{Body, BodyPartHandle, ColliderDesc, DefaultBodySet, DefaultColliderSet, Ground, RigidBodyDesc};
use nphysics2d::world::{DefaultMechanicalWorld, DefaultGeometricalWorld};

use nphysics_testbed2d::Testbed;

pub fn init_world(testbed: &mut Testbed) { 
    // Initialize simulation parameters:
    const G: f32 = -9.81; // Gravitational acceleration [m/s^2]
    const M_C: f32 = 1.5; // Mass of cart body [kg]
    const M_P: f32 = 1.0; // Mass of pole body [kg]
    const L_P: f32 = 2.0; // Lenght of pole [m]
    const J_P: f32 = M_P * L_P / 6.0; // m * l^2 / 12 /(l / 2) = J_p / (l / 2)
    const ZETA: f32 = 0.7; // Damping coefficient
    const W_N_A: f32 = 100.0; // natural frequency of angular dynamics
    const W_N_L: f32 = 5.0; // natural frequency of linear dynamics
    // World
    let mechanical_world = DefaultMechanicalWorld::new(Vector2::new(0.0, G));
    let geometrical_world = DefaultGeometricalWorld::new();
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

    // For handling collisions, add a buffer zone
    //let geom = ShapeHandle::new(Cuboid::new(Vector2::repeat(0.2)));
    //let collider_desc = ColliderDesc::new(geom).density(1.0);
    let cart_anchor = Point2::new(0.0, 0.5);
    let pole_anchor = Point2::new(0.0, 2.0);

    // Cart body 
    let cart_body = RigidBodyDesc::new()
        .mass(2.0)
        .translation(cart_anchor.coords)
        .build();
        //.velocity(Velocity::linear(2.0, 0.0))
    let cart_handle = bodies.insert(cart_body);
    let cart_geom = ShapeHandle::new(Cuboid::new(Vector2::new(2.0, 1.0)));
    let cart_collider = ColliderDesc::new(cart_geom)
        .collision_groups(cart_group)
        .density(1.0)
        .build(BodyPartHandle(cart_handle, 0));
    colliders.insert(cart_collider);

    // Pole body
    let pole_body = RigidBodyDesc::new()
        .mass(1.0)
        .translation(pole_anchor.coords)
        .build();
    let pole_handle = bodies.insert(pole_body);
    let pole_geom = ShapeHandle::new(Cuboid::new(Vector2::new(0.1, 2.0)));
    let pole_collider = ColliderDesc::new(pole_geom)
        .collision_groups(pole_group)
        .density(1.0)
        .build(BodyPartHandle(pole_handle, 0));
    colliders.insert(pole_collider);

    // Add revolute constraint between cart and pole
    let revolute_constraint = RevoluteConstraint::new(
        BodyPartHandle(cart_handle, 0),
        BodyPartHandle(pole_handle, 0),
        Point2::new(0.0, 1.0),
        Point2::new(0.0, -2.0),
    );
    joint_constraints.insert(revolute_constraint);

    // Add prismatic joint between the ground and the cart body
    let prismatic_constraint = PrismaticConstraint::new(
        BodyPartHandle(ground_handle, 0), // parent (ground)
        BodyPartHandle(cart_handle, 0), // child (cart)
        cart_anchor,
        Vector2::x_axis(),
        Point2::origin(),
    );
    joint_constraints.insert(prismatic_constraint);


    // DON'T DELETE THIS:
    // .add_callback( move | _, geometrical_world, bodies, colliders, graphics, _|
    testbed.add_callback(move |_, _, bodies, _, _, _| {
        let pole = bodies.rigid_body(pole_handle).unwrap();
        let w = pole.velocity().angular;
        let phi = pole.position().rotation.angle();
        let mut pole_input = 0.0;

        if phi.abs() < 3.1 / 2.0 { 
            let w_dot_ref = - 2.0 * ZETA * W_N_A * w - W_N_A.powi(2) * phi; // w_ref = 0, phi_ref = 0
            pole_input = J_P * w_dot_ref / phi.cos() + M_P * G * phi.tan();
        }

        let cart = bodies.rigid_body_mut(cart_handle).unwrap();
        let p = cart.position().translation.vector[0];
        let v = cart.velocity().linear.data[0];
        let cart_input = M_C * (- 2.0 * ZETA * W_N_L * v - W_N_L.powi(2) * p);

        let force = Force::linear(Vector2::new(- cart_input + pole_input, 0.0));
        cart.apply_force(0, &force, ForceType::Force, false);
    });

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
    testbed.look_at(Point2::new(0.0, 0.0), 25.0);
}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Kinematic body", init_world)]);
    testbed.run();
}
