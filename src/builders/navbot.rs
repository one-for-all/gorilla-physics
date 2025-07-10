use na::{vector, Matrix3};

use crate::{
    collision::{mesh::Mesh, sphere::Sphere},
    inertia::SpatialInertia,
    joint::{floating::FloatingJoint, Joint},
    mechanism::MechanismState,
    rigid_body::{Collider, RigidBody},
    spatial::transform::Transform3D,
    WORLD_FRAME,
};

fn build_navbot_motor_body(mesh: Mesh, frame: &str) -> RigidBody {
    let m = 0.041;
    let r = 0.01385;
    let d = 0.02;

    let ixx = m * (3.0 * r * r + d * d) / 12.0;
    let izz = ixx;
    let iyy = m * r * r / 2.0;
    let moment = Matrix3::from_diagonal(&vector![ixx, iyy, izz]);

    let cross_part = vector![0., 0., 0.];
    let spatial_inertia = SpatialInertia::new(moment, cross_part, m, frame);

    let collider = Collider::new_sphere(Sphere::new(r));

    RigidBody::new_collider_and_visual(collider, mesh, spatial_inertia)
}

pub fn build_navbot_motor(mesh: Mesh) -> MechanismState {
    let frame = "motor";
    let body = build_navbot_motor_body(mesh, frame);

    let body_to_world = Transform3D::identity(frame, WORLD_FRAME);

    let treejoints = vec![Joint::FloatingJoint(FloatingJoint::new(body_to_world))];
    let bodies = vec![body];
    MechanismState::new(treejoints, bodies)
}
