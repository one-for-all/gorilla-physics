use na::{vector, Matrix3, Vector, Vector3};

use crate::{
    collision::mesh::Mesh,
    inertia::SpatialInertia,
    joint::{revolute::RevoluteJoint, Joint},
    mechanism::MechanismState,
    rigid_body::RigidBody,
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

    RigidBody::new_mesh(mesh, spatial_inertia, false)
}

pub fn build_navbot_motor(mesh: Mesh) -> MechanismState {
    let frame = "motor";
    let body = build_navbot_motor_body(mesh, frame);

    // let body_to_world = Transform3D::identity(frame, WORLD_FRAME);
    let body_to_world = Transform3D::move_z(frame, WORLD_FRAME, 0.03);

    let treejoints = vec![Joint::RevoluteJoint(RevoluteJoint::new(
        body_to_world,
        Vector3::y_axis(),
    ))];
    let bodies = vec![body];
    MechanismState::new(treejoints, bodies)
}
