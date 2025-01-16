use na::{dvector, vector, Matrix3, Matrix4};
use wasm_bindgen::prelude::*;

use crate::{helpers::build_double_pendulum, transform::Transform3D, types::Float, PI};

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

    let rod1_to_world = Matrix4::identity();
    let rod2_to_rod1 = Transform3D::move_x(l);
    let axis = vector![0.0, -1.0, 0.0]; // axis of joint rotation

    let mut state = build_double_pendulum(
        &m,
        &moment,
        &cross_part,
        &rod1_to_world,
        &rod2_to_rod1,
        &axis,
    );

    let q_init = dvector![-PI / 2.0 + 0.1, 0.];
    let v_init = dvector![0., 0.];
    state.update(&q_init, &v_init);

    InterfaceMechanismState(state)
}
