use super::InterfaceMechanismState;
use crate::helpers::build_cart_pole;
use crate::joint::ToJointPositionVec;
use crate::joint::ToJointVelocityVec;
use crate::types::Float;
use na::Vector3;
use nalgebra::{vector, Matrix3};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub fn createCartPole(length: Float) -> InterfaceMechanismState {
    let m_cart = 3.0;
    let l_cart = 1.0;
    let moment_x = 0.0;
    let moment_y = m_cart * l_cart * l_cart / 12.0;
    let moment_z = m_cart * l_cart * l_cart / 12.0;
    let moment_cart = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
    let cross_part_cart = vector![0.0, 0.0, 0.0];

    let m_pole = 5.0;
    let l_pole = length;
    let moment_x = m_pole * l_pole * l_pole;
    let moment_y = m_pole * l_pole * l_pole;
    let moment_z = 0.0;
    let moment_pole = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
    let cross_part_pole = vector![0.0, 0.0, -l_pole * m_pole];
    let axis_pole = -Vector3::y_axis();

    let mut state = build_cart_pole(
        m_cart,
        m_pole,
        &moment_cart,
        &moment_pole,
        &cross_part_cart,
        &cross_part_pole,
        axis_pole,
    );

    let q_init = vec![0.0, 0.1].to_joint_pos_vec(); // -(PI / 2.0 + 1.5)
    let v_init = vec![0.0, 0.0].to_joint_vel_vec();
    state.update(&q_init, &v_init);

    InterfaceMechanismState { inner: state }
}
