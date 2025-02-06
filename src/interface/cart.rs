use super::InterfaceMechanismState;
use crate::helpers::build_cart;
use crate::joint::ToJointPositionVec;
use crate::joint::ToJointVelocityVec;
use nalgebra::{dvector, vector, Matrix3};
use wasm_bindgen::prelude::*;

use crate::types::Float;

#[wasm_bindgen]
pub fn createCart(length: Float) -> InterfaceMechanismState {
    let m = 3.0;
    let l = length;

    let moment_x = 0.0;
    let moment_y = m * l * l / 12.0;
    let moment_z = m * l * l / 12.0;
    let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
    let cross_part = vector![0.0, 0.0, 0.0];
    let axis = vector![1.0, 0.0, 0.0];

    let mut state = build_cart(&m, &moment, &cross_part, &axis);

    let q_init = vec![0.0].to_joint_pos_vec();
    let v_init = vec![1.0].to_joint_vel_vec();
    state.update(&q_init, &v_init);

    InterfaceMechanismState(state)
}
