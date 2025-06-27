use na::{vector, UnitQuaternion, Vector3};
use wasm_bindgen::{prelude::wasm_bindgen, JsCast};
use wasm_bindgen_futures::JsFuture;
use web_sys::{window, Response};

use crate::{
    helpers::build_rigid_mesh_box,
    joint::{JointPosition, JointVelocity},
    mesh::Mesh,
    spatial::{pose::Pose, spatial_vector::SpatialVector},
    PI,
};

use super::{util::read_web_file, InterfaceMechanismState};

/// Create a rigid body box from a box mesh file
#[wasm_bindgen]
pub async fn createRigidMeshBox() -> InterfaceMechanismState {
    let buf = read_web_file("box.mesh").await;

    let mesh = Mesh::new_from_str(&buf);

    let l = 2.0;
    let mut state = build_rigid_mesh_box(mesh, l);

    let rot_init = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), PI / 2.0);
    let q_init = vec![JointPosition::Pose(Pose {
        rotation: rot_init,
        translation: vector![0.0, 0.0, 2.0],
    })];
    let v_init = vec![JointVelocity::Spatial(SpatialVector {
        angular: vector![0., 0., 0.],
        linear: rot_init.inverse() * vector![1.0, 0.0, 0.0],
    })];
    state.update(&q_init, &v_init);

    InterfaceMechanismState { inner: state }
}
