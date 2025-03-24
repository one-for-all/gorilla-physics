use na::{vector, Matrix3, Matrix4, UnitQuaternion};
use wasm_bindgen::prelude::wasm_bindgen;

use crate::{
    contact::ContactPoint,
    control::quadruped_control::QuadrupedTrottingController,
    helpers::build_quadruped,
    inertia::SpatialInertia,
    joint::{floating::FloatingJoint, revolute::RevoluteJoint, Joint, JointPosition},
    mechanism::MechanismState,
    pose::Pose,
    rigid_body::RigidBody,
    transform::{Matrix4Ext, Transform3D},
    types::Float,
    PI, WORLD_FRAME,
};

use super::InterfaceMechanismState;

#[wasm_bindgen]
pub fn createQuadruped(l_leg: Float, default_z: Float) -> InterfaceMechanismState {
    let mut state = build_quadruped();

    let initial_x = -1.2;
    state.set_joint_q(
        1,
        JointPosition::Pose(Pose {
            rotation: UnitQuaternion::identity(),
            translation: vector![initial_x, 0., default_z],
        }),
    );

    let default_foot_stance = vec![
        vector![0., 0., -default_z],
        vector![0., 0., -default_z],
        vector![0., 0., -default_z],
        vector![0., 0., -default_z],
    ];
    let default_joint_angles =
        QuadrupedTrottingController::inverse_kinematics(&default_foot_stance, l_leg);

    for (leg_index, hip_knee_angles) in default_joint_angles.iter().enumerate() {
        let hip_index = leg_index * 2 + 1;
        let knee_index = hip_index + 1;

        let hip_angle = hip_knee_angles[0];
        let knee_angle = hip_knee_angles[1];

        state.set_joint_q(hip_index + 1, JointPosition::Float(hip_angle));
        state.set_joint_q(knee_index + 1, JointPosition::Float(knee_angle));
    }

    InterfaceMechanismState { inner: state }
}
