use na::{vector, UnitQuaternion, Vector3};
use wasm_bindgen::prelude::wasm_bindgen;

use crate::{
    collision::mesh::Mesh,
    helpers::{
        build_rigid_mesh_box, build_tetrahedron, build_two_cubes, build_two_rigid_mesh_boxes,
        build_two_tetrahedron,
    },
    joint::{JointPosition, JointVelocity},
    spatial::{pose::Pose, spatial_vector::SpatialVector},
};

use super::{util::read_web_file, InterfaceMechanismState};

#[wasm_bindgen]
pub async fn createRigidTetrahedron() -> InterfaceMechanismState {
    let buf = read_web_file("tetrahedron.obj").await;

    let mesh = Mesh::new_from_obj(&buf);

    let l = 1.0;
    let mut state = build_tetrahedron(mesh, l);

    let rot_init = UnitQuaternion::identity();
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

/// Create two rigid body tetrahedrons from a tetrahedron mesh file
#[wasm_bindgen]
pub async fn createTwoRigidTetrahedron() -> InterfaceMechanismState {
    let buf = read_web_file("tetrahedron.obj").await;
    let mesh = Mesh::new_from_obj(&buf);

    let l = 1.0;
    let mut state = build_two_tetrahedron(mesh, l);

    let q_init = vec![
        JointPosition::Pose(Pose {
            rotation: UnitQuaternion::identity(),
            translation: vector![0.0, 0.0, 0.0],
        }),
        JointPosition::Pose(Pose {
            rotation: UnitQuaternion::identity(),
            translation: vector![0.2, 0.2, 2.0 * l],
        }),
    ];
    state.update_q(&q_init);

    InterfaceMechanismState { inner: state }
}

/// Create a rigid body box from a box mesh file
#[wasm_bindgen]
pub async fn createRigidMeshBox() -> InterfaceMechanismState {
    let buf = read_web_file("box.mesh").await;
    let mesh = Mesh::new_from_mesh(&buf);

    let l = 2.0;
    let mut state = build_rigid_mesh_box(mesh, l);

    let rot_init = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.0);
    let q_init = vec![JointPosition::Pose(Pose {
        rotation: rot_init,
        translation: vector![0.0, 0.0, 1.0],
    })];
    let v_init = vec![JointVelocity::Spatial(SpatialVector {
        angular: vector![0., 0., 0.],
        linear: rot_init.inverse() * vector![0.0, 0.0, 0.0],
    })];
    state.update(&q_init, &v_init);

    InterfaceMechanismState { inner: state }
}

/// Create two rigid body boxes from a box mesh file
#[wasm_bindgen]
pub async fn createTwoRigidMeshBoxes() -> InterfaceMechanismState {
    let buf = read_web_file("box.mesh").await;
    let mesh = Mesh::new_from_mesh(&buf);

    let l = 2.0;
    let mut state = build_two_rigid_mesh_boxes(mesh, l);

    let q_init = vec![
        JointPosition::Pose(Pose {
            rotation: UnitQuaternion::identity(),
            translation: vector![0.0, 0.0, l / 2.0],
        }),
        JointPosition::Pose(Pose {
            rotation: UnitQuaternion::identity(),
            translation: vector![0.0, 0.0, l + 2.0],
        }),
    ];
    state.update_q(&q_init);

    InterfaceMechanismState { inner: state }
}

#[wasm_bindgen]
pub async fn createTwoRigidCubes() -> InterfaceMechanismState {
    let buf = read_web_file("cube.obj").await;
    let mesh = Mesh::new_from_obj(&buf);

    let l = 1.0;
    let mut state = build_two_cubes(mesh, l);

    let rot_init = UnitQuaternion::identity();
    let q_init = vec![
        JointPosition::Pose(Pose {
            rotation: UnitQuaternion::identity(),
            translation: vector![0.0, 0.0, 0.0],
        }),
        JointPosition::Pose(Pose {
            rotation: rot_init,
            translation: vector![0., 0., 2.0 * l],
        }),
    ];
    state.update_q(&q_init);

    InterfaceMechanismState { inner: state }
}
