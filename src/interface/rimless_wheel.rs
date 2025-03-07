use crate::{
    contact::ContactPoint,
    inertia::SpatialInertia,
    joint::{floating::FloatingJoint, Joint, JointPosition, JointVelocity},
    mechanism::MechanismState,
    pose::Pose,
    rigid_body::RigidBody,
    spatial_vector::SpatialVector,
    transform::Transform3D,
    types::Float,
    PI,
};

use super::InterfaceMechanismState;
use na::{dvector, vector, Matrix3, Rotation3, UnitQuaternion, Vector3};
use wasm_bindgen::prelude::wasm_bindgen;

#[wasm_bindgen]
pub fn createRimlessWheel(r_body: Float, n_foot: usize) -> InterfaceMechanismState {
    let m_body = 10.0;
    let r_body = r_body;

    let l = 10.0;

    let moment_x = 2.0 / 5.0 * m_body * r_body * r_body;
    let moment = Matrix3::from_diagonal(&vector![moment_x, moment_x, moment_x]);
    let cross_part = vector![0.0, 0.0, 0.0];

    let body_frame = "body";
    let world_frame = "world";
    let body_to_world = Transform3D::identity(&body_frame, &world_frame);

    let body = RigidBody::new(SpatialInertia {
        frame: body_frame.to_string(),
        mass: m_body,
        moment,
        cross_part,
    });

    let alpha = 2.0 * PI / n_foot as Float / 2.0;

    let treejoints = vec![Joint::FloatingJoint(FloatingJoint::new(body_to_world))];
    let bodies = dvector![body];
    let mut state = MechanismState::new(treejoints, bodies);

    for i in 0..n_foot {
        let rotation = Rotation3::from_axis_angle(&Vector3::y_axis(), i as Float * 2.0 * alpha);
        let location = rotation * Vector3::new(0., 0., -l);
        state.add_contact_point(&ContactPoint {
            frame: body_frame.to_string(),
            location,
        });
    }

    let q_init = vec![JointPosition::Pose(Pose {
        rotation: UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.0), // Note: cases where angle = n * PI/2.0 would crash the simulation.
        translation: vector![0.0, 0.0, 0.0],
    })];
    let v_init = vec![JointVelocity::Spatial(SpatialVector {
        angular: vector![0.0, 0.0, 0.0],
        linear: vector![1.0, 0.0, 0.0],
    })];
    state.update(&q_init, &v_init);

    InterfaceMechanismState { inner: state }
}
