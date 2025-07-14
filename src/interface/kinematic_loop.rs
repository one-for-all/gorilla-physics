use wasm_bindgen::prelude::wasm_bindgen;

use crate::{
    helpers::build_four_bar_linkage, interface::InterfaceMechanismState, joint::JointPosition, PI,
};

#[wasm_bindgen]
pub async fn createFourBarLinkage() -> InterfaceMechanismState {
    let mut state = build_four_bar_linkage(1.0, 10.0);

    let angle = PI / 2.0; // PI - 0.1;
    let q = vec![
        JointPosition::Float(-angle),
        JointPosition::Float(-angle),
        JointPosition::Float(angle),
    ];
    state.update_q(&q);

    InterfaceMechanismState { inner: state }
}
