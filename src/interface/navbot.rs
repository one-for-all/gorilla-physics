use wasm_bindgen::prelude::wasm_bindgen;

use crate::{builders::navbot::build_navbot_motor, collision::mesh::Mesh};

use super::{util::read_web_file, InterfaceMechanismState};

#[wasm_bindgen]
pub async fn createNavbotMotor() -> InterfaceMechanismState {
    let buf = read_web_file("navbot/quanum_gimbal_motor_2208_base.obj").await;
    let mesh = Mesh::new_from_obj(&buf);

    let state = build_navbot_motor(mesh);

    InterfaceMechanismState { inner: state }
}
