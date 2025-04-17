use wasm_bindgen::prelude::wasm_bindgen;

use crate::helpers::build_pusher;

use super::InterfaceMechanismState;

#[wasm_bindgen]
pub fn createPusher() -> InterfaceMechanismState {
    let state = build_pusher();

    InterfaceMechanismState { inner: state }
}
