use super::InterfaceMechanismState;
use crate::helpers::build_cube;
use crate::joint::JointPosition;
use crate::joint::JointVelocity;
use crate::pose::Pose;
use crate::spatial_vector::SpatialVector;
use na::UnitQuaternion;
use na::Vector3;
use nalgebra::vector;
use wasm_bindgen::prelude::*;

use crate::types::Float;

#[wasm_bindgen]
pub fn createCube(length: Float) -> InterfaceMechanismState {
    let m = 3.0;
    let l = length;
    let mut state = build_cube(m, l);

    let q_init = vec![JointPosition::Pose(Pose {
        rotation: UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.0),
        translation: vector![0.0, 0.0, 0.0],
    })];
    let v_init = vec![JointVelocity::Spatial(SpatialVector {
        angular: vector![0.0, 5.0, 0.0],
        linear: vector![1.0, 0.0, 0.0],
    })];
    state.update(&q_init, &v_init);

    InterfaceMechanismState { inner: state }
}
