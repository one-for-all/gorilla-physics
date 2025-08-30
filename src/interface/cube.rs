use super::InterfaceMechanismState;
use crate::helpers::build_cube;
use crate::joint::JointPosition;
use crate::joint::JointVelocity;
use crate::spatial::pose::Pose;
use crate::spatial::spatial_vector::SpatialVector;
use crate::PI;
use na::UnitQuaternion;
use na::Vector3;
use nalgebra::vector;
use wasm_bindgen::prelude::*;

use crate::types::Float;

#[wasm_bindgen]
pub async fn createCube(length: Float) -> InterfaceMechanismState {
    let m = 3.0;
    let l = length;
    let mut state = build_cube(m, l);

    let rot = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.);
    let q_init = vec![JointPosition::Pose(Pose {
        rotation: rot,
        translation: vector![0.0, 0.0, l / 2.],
    })];
    let v_init = vec![JointVelocity::Spatial(SpatialVector {
        angular: vector![0.0, 0.0, 0.0],
        linear: rot.inverse() * vector![0.0, 0.0, 0.0],
    })];
    state.update(&q_init, &v_init);

    InterfaceMechanismState { inner: state }
}
