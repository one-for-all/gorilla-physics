use na::{vector, UnitQuaternion};
use wasm_bindgen::prelude::wasm_bindgen;

use crate::{
    builders::navbot::build_navbot_motor, collision::mesh::Mesh, joint::JointPosition,
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
