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
        .translation(Vector2::y() * 2.0)
        .build(BodyPartHandle(ground_handle, 0));

    colliders.insert(co);

    // For handling collisions, add a buffer zone
    let geom = ShapeHandle::new(Cuboid::new(Vector2::repeat(0.2)));
    let collider_desc = ColliderDesc::new(geom).density(1.0);

    // Cart body 
    let cart_body = RigidBodyDesc::new()
        .velocity(Velocity::linear(2.0, 0.0))
        .build();
    let cart_handle = bodies.insert(cart_body);
    let cart_geom = ShapeHandle::new(Cuboid::new(Vector2::new(2.0, 1.0)));
    let cart_collider = ColliderDesc::new(cart_geom)
        .collision_groups(cart_group)
        .density(1.0)
        .build(BodyPartHandle(cart_handle, 0));
    colliders.insert(cart_collider);

    // Pole body
    let pole_body = RigidBodyDesc::new()
        .translation(Vector2::new(0.0, 2.0))
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
        Point2::new(0.0, -2.2),
    );

    joint_constraints.insert(revolute_constraint);

    // TODO
    // Add prismatic constraint to constrain the motion of the cart
    //let constraint = PrismaticConstraint::new(


    // Add force on the cart as an input
    //pub struct InputForce {
    //    parts: Vec<BodyPartHandle>,
    //    center: Point2<f32>
    //}

    //impl InputForce {
    //    pub fn new(center: Point2<f32>, parts: Vec<BodyPartHandle>) -> Self {
    //        InputForce {
    //            parts,
    //            center,
    //        }
    //    }

    //    pub fn add_body(&mut self, body: BodyPartHandle) {
    //        self.pars.push(body)
    //    }
    //}

    //impl ForceGenerator<f32> for InputForce {
    //    fn apply(&mut self, _: &IntegrationParameters<f32>, bodies: &mut BodySet<f32>, magnitude: f32) -> bool {
    //        for handle in &self.parts {
    //            if let Some(body) = bodies.body_mut(handle.0) {
    //                let part = body.part(handle.1).unwrap();

    //                let force = Force::linear(magnitude);
    //                body.apply_force(handle.1, &force, ForceType::Force, false);
    //            }
    //        }
    //        true
    //    }
    //}

    // .add_callback( move | _, geometrical_world, bodies, colliders, graphics, _|
    testbed.add_callback(move |_, _, bodies, _, _, _| {
        let force = Force::linear(Vector2::new(-250.0, 0.0));
        if let Some(platform) = bodies.rigid_body_mut(cart_handle) {
            let platform_x = platform.position().translation.vector.x;

            let mut vel = *platform.velocity();

            if platform_x >= 10.0 {
                vel.linear.x = - vel.linear.x;
            }
            if platform_x <= 10.0 {
                vel.linear.x = - vel.linear.x;
            }

            platform.apply_force(0, &force, ForceType::Force, false);

            platform.set_velocity(vel);
        }
        //if let Some(rod) = bodies.rigid_body_mut(pole_handle) {
        //    println!("{}", rod.velocity().angular);
        //}
    });

    /*
     * Run the simulation.
     */
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
