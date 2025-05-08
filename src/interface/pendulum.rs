use na::{vector, Matrix3, Matrix4};
use wasm_bindgen::prelude::wasm_bindgen;

use crate::{contact::ContactPoint, helpers::build_pendulum, types::Float};

use super::InterfaceMechanismState;

/// Create a simple pendulum with a contact point at the end of the rod
#[wasm_bindgen]
pub fn createSimplePendulum(mass: Float, length: Float) -> InterfaceMechanismState {
    let m = mass;
    let l = length;

    let moment_x = 0.0;
    let moment_y = m * l * l;
    let moment_z = m * l * l;
    let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
    let cross_part = vector![m * l, 0., 0.];

    let rod_to_world = Matrix4::identity();
    let axis = vector![0., 1., 0.];

    let mut state = build_pendulum(&m, &moment, &cross_part, &rod_to_world, &axis);
    state.add_contact_point(ContactPoint::new("rod", vector![l, 0., 0.]));

    InterfaceMechanismState { inner: state }
}
