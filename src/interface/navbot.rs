use na::{vector, UnitQuaternion};
use wasm_bindgen::prelude::wasm_bindgen;

use crate::{
    builders::navbot::{build_balancing_bot, build_navbot_motor},
    collision::mesh::Mesh,
    joint::JointPosition,
    spatial::pose::Pose,
};

use super::{util::read_web_file, InterfaceMechanismState};

#[wasm_bindgen]
pub async fn createNavbotMotor() -> InterfaceMechanismState {
    let buf = read_web_file("navbot/quanum_gimbal_motor_2208_base.obj").await;
    let mesh = Mesh::new_from_obj(&buf);

    let mut state = build_navbot_motor(mesh);

    let q_init = vec![JointPosition::Pose(Pose {
        rotation: UnitQuaternion::identity(),
        translation: vector![0., 0., 0.01385],
    })];
    state.update_q(&q_init);

    InterfaceMechanismState { inner: state }
}

#[wasm_bindgen]
pub async fn createBalancingBot() -> InterfaceMechanismState {
    let mut state = build_balancing_bot();

    let h_body = 0.025;
    let h_offset = h_body * 2.0;
    let r_wheel = 0.02;
    let q_body_init = JointPosition::Pose(Pose {
        // rotation: UnitQuaternion::identity(),
        rotation: UnitQuaternion::from_euler_angles(0., 0., 0.),
        translation: vector![0., 0.1, h_offset + r_wheel],
    });
    state.set_joint_q(1, q_body_init);

    InterfaceMechanismState { inner: state }
}
