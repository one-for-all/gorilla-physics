use crate::joint::ToJointVelocityVec;
use na::{vector, Isometry3, Matrix3, Matrix4};
use wasm_bindgen::prelude::*;

use crate::spatial::transform::Matrix4Ext;
use crate::{helpers::build_double_pendulum, joint::ToJointPositionVec, types::Float, PI};

use super::InterfaceMechanismState;

#[wasm_bindgen]
pub fn createDoublePendulum(length: Float) -> InterfaceMechanismState {
    let m = 1.0;
    let l = length;

    let moment_x = 0.0;
    let moment_y = m * l * l;
    let moment_z = m * l * l;
    let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
    let cross_part = vector![m * l, 0., 0.];

    let rod1_to_world = Isometry3::identity();
    let rod2_to_rod1 = Isometry3::translation(l, 0., 0.);
    let axis = vector![0.0, -1.0, 0.0]; // axis of joint rotation

    let mut state = build_double_pendulum(
        &m,
        &moment,
        &cross_part,
        &rod1_to_world,
        &rod2_to_rod1,
        &axis,
    );

    let q_init = vec![-PI / 2.0 + 0.1, 0.].to_joint_pos_vec();
    let v_init = vec![0., 0.].to_joint_vel_vec();
    state.update(&q_init, &v_init);

    InterfaceMechanismState { inner: state }
}
