use wasm_bindgen::prelude::wasm_bindgen;

use crate::{
    builders::biped_builder::build_biped,
    interface::InterfaceMechanismState,
    joint::{Joint, JointPosition, JointVelocity},
    PI,
};

#[wasm_bindgen]
pub async fn createBiped() -> InterfaceMechanismState {
    let mut state = build_biped();

    // let q_init = vec![
    //     JointPosition::None,
    //     JointPosition::Float(0.1),
    //     JointPosition::Float(0.1),
    // ];
    // state.update_q(&q_init);

    state.set_joint_v(2, JointVelocity::Float(0.5));
    state.set_joint_q(3, JointPosition::Float(-PI / 2.));

    InterfaceMechanismState { inner: state }
}
