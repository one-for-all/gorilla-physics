use na::vector;
use wasm_bindgen::prelude::wasm_bindgen;

use crate::{
    builders::kinematic_loop_builder::{
        build_four_bar_linkage, build_four_bar_linkage_with_base, build_mock_navbot,
    },
    interface::InterfaceMechanismState,
    joint::{JointPosition, JointVelocity},
    spatial::{pose::Pose, spatial_vector::SpatialVector},
    PI,
};

#[wasm_bindgen]
pub async fn createFourBarLinkage() -> InterfaceMechanismState {
    let mut state = build_four_bar_linkage(1.0, 1.0);

    let angle = PI - 0.1; // PI / 2.0; //
    let q = vec![
        JointPosition::Float(-angle),
        JointPosition::Float(-angle),
        JointPosition::Float(angle),
    ];
    state.update_q(&q);

    InterfaceMechanismState { inner: state }
}

#[wasm_bindgen]
pub async fn createFourBarLinkageWithBase() -> InterfaceMechanismState {
    let mut state = build_four_bar_linkage_with_base(1.0, 1.0);

    let angle = 0.; // PI / 4.0; // PI - 0.1;
    let q = vec![
        JointPosition::Float(-PI / 4.0),
        JointPosition::Float(-angle),
        JointPosition::Float(-angle),
        JointPosition::Float(angle),
    ];
    state.update_q(&q);

    state.set_joint_v(1, JointVelocity::Float(0.2));

    InterfaceMechanismState { inner: state }
}

#[wasm_bindgen]
pub async fn createMockNavbot() -> InterfaceMechanismState {
    let mut state = build_mock_navbot(1.0);

    let angle = PI / 3.0; // PI / 4.0; // PI - 0.1;
    let q = vec![
        JointPosition::Pose(Pose::identity()),
        JointPosition::Float(-angle),
        JointPosition::Float(-angle),
        JointPosition::Float(angle),
        JointPosition::Float(angle),
        JointPosition::Float(angle),
        JointPosition::Float(-angle),
    ];
    state.update_q(&q);

    state.set_joint_v(
        1,
        JointVelocity::Spatial(SpatialVector {
            angular: vector![0., 0., 0.1],
            linear: vector![0., 0., 0.],
        }),
    );

    state.set_joint_v(
        1,
        JointVelocity::Spatial(SpatialVector {
            angular: vector![0., 0., 0.5],
            linear: vector![0., 0., 2.],
        }),
    );

    InterfaceMechanismState { inner: state }
}
