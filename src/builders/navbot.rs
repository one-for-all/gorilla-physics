use std::iter::Rev;

use na::{vector, Matrix3, Vector3};

use crate::{
    collision::{mesh::Mesh, sphere::Sphere},
    contact::ContactPoint,
    inertia::SpatialInertia,
    joint::{floating::FloatingJoint, revolute::RevoluteJoint, Joint},
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

pub fn build_balancing_bot() -> MechanismState {
    let body_frame = "body";
    let w_body = 0.06;
    let d_body = 0.05;
    let h_body = 0.025;
    let m_body = 0.5;
    let mut body = RigidBody::new_cuboid(m_body, w_body, d_body, h_body, body_frame);
    body.add_cuboid_contacts(w_body, d_body, h_body);

    let body_to_world = Transform3D::identity(body_frame, WORLD_FRAME);

    let m_wheel = 0.05;
    let r_wheel = 0.02;
    let h_offset = -2.0 * h_body;
    let wheel_left_frame = "wheel_left";
    let wheel_left = RigidBody::new_sphere(m_wheel, r_wheel, wheel_left_frame);
    let wheel_left_to_body =
        Transform3D::move_xyz(wheel_left_frame, body_frame, -w_body / 2.0, 0., h_offset);

    let wheel_right_frame = "wheel_right";
    let wheel_right = RigidBody::new_sphere(m_wheel, r_wheel, wheel_right_frame);
    let wheel_right_to_body =
        Transform3D::move_xyz(wheel_right_frame, body_frame, w_body / 2.0, 0., h_offset);

    let treejoints = vec![
        Joint::FloatingJoint(FloatingJoint::new(body_to_world)),
        Joint::RevoluteJoint(RevoluteJoint::new(wheel_left_to_body, Vector3::x_axis())),
        Joint::RevoluteJoint(RevoluteJoint::new(wheel_right_to_body, Vector3::x_axis())),
    ];
    let bodies = vec![body, wheel_left, wheel_right];
    MechanismState::new(treejoints, bodies)
}
