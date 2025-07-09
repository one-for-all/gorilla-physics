use na::{vector, UnitQuaternion, Vector3};
use wasm_bindgen::prelude::wasm_bindgen;

use crate::{
    builders::build_so101,
    collision::mesh::Mesh,
    helpers::{
        build_rigid_mesh_box, build_tetrahedron, build_two_cubes, build_two_rigid_mesh_boxes,
        build_two_tetrahedron,
    },
    joint::{JointPosition, JointVelocity},
    spatial::{pose::Pose, spatial_vector::SpatialVector},
    PI,
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
            rotation: UnitQuaternion::from_euler_angles(0., PI / 2.0, 0.),
            translation: vector![0.0, 0.0, 2.0],
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

#[wasm_bindgen]
pub async fn createSO101() -> InterfaceMechanismState {
    let base_buf = read_web_file("so101/base_so101_v2.obj").await;
    let base_mesh = Mesh::new_from_obj(&base_buf);

    let shoulder_buf = read_web_file("so101/rotation_pitch_so101_v1.obj").await;
    let shoulder_mesh = Mesh::new_from_obj(&shoulder_buf);

    let upper_arm_buf = read_web_file("so101/upper_arm_so101_v1.obj").await;
    let upper_arm_mesh = Mesh::new_from_obj(&upper_arm_buf);

    let lower_arm_buf = read_web_file("so101/under_arm_so101_v1.obj").await;
    let lower_arm_mesh = Mesh::new_from_obj(&lower_arm_buf);

    let wrist_buf = read_web_file("so101/wrist_roll_pitch_so101_v2.obj").await;
    let wrist_mesh = Mesh::new_from_obj(&wrist_buf);

    let gripper_buf = read_web_file("so101/wrist_roll_follower_so101_v1.obj").await;
    let gripper_mesh = Mesh::new_from_obj(&gripper_buf);

    let state = build_so101(
        base_mesh,
        shoulder_mesh,
        upper_arm_mesh,
        lower_arm_mesh,
        wrist_mesh,
        gripper_mesh,
    );

    InterfaceMechanismState { inner: state }
}
