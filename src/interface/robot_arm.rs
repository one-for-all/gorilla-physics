use wasm_bindgen::prelude::wasm_bindgen;

use crate::helpers::{build_gripper, build_pusher};

use super::InterfaceMechanismState;

#[wasm_bindgen]
pub async fn createPusher() -> InterfaceMechanismState {
    let state = build_pusher();

    InterfaceMechanismState { inner: state }
}

#[wasm_bindgen]
pub fn createGripper() -> InterfaceMechanismState {
    let state = build_gripper();

    InterfaceMechanismState { inner: state }
}
