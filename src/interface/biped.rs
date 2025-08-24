use na::{vector, zero, UnitQuaternion, UnitVector3, Vector3};
use wasm_bindgen::prelude::wasm_bindgen;

use crate::{
    builders::{
        biped_builder::build_biped,
        leg_builder::{build_leg, build_leg_from_foot},
    },
    flog,
    interface::InterfaceMechanismState,
    joint::{Joint, JointPosition, JointVelocity},
    spatial::pose::Pose,
    PI,
};

#[wasm_bindgen]
pub async fn createBiped() -> InterfaceMechanismState {
    let mut state = build_biped();

    // let q_init = vec![
    //     JointPosition::None,
    //     JointPosition::Float(0.1),
    //     JointPosition::Float(0.1),
    // ];
    // state.update_q(&q_init);

    // state.set_joint_v(2, JointVelocity::Float(0.5));
    // state.set_joint_q(3, JointPosition::Float(-PI / 2.));

    InterfaceMechanismState { inner: state }
}

#[wasm_bindgen]
pub async fn createLeg() -> InterfaceMechanismState {
    let mut state = build_leg();

    let thigh_angle = -PI / 4.;
    let calf_angle = PI / 2.;
    let foot_angle = -PI / 4.;
    let q_init = vec![
        JointPosition::Pose(Pose {
            rotation: UnitQuaternion::from_axis_angle(&Vector3::x_axis(), thigh_angle),
            translation: zero(),
        }),
        JointPosition::Float(calf_angle),
        JointPosition::Float(foot_angle),
    ];

    state.update_q(&q_init);

    // Set the leg to appropriate height
    let h_foot = 0.05;
    let foot_height = state.poses().last().unwrap().translation.z;
    state.set_joint_q(
        1,
        JointPosition::Pose(Pose {
            rotation: UnitQuaternion::from_axis_angle(&Vector3::x_axis(), thigh_angle),
            translation: vector![0., 0., -foot_height + h_foot / 2.],
        }),
    );

    InterfaceMechanismState { inner: state }
}

#[wasm_bindgen]
pub async fn createLegFromFoot() -> InterfaceMechanismState {
    let mut state = build_leg_from_foot();

    let calf_angle = PI / 4. - 0.4;
    let thigh_angle = -PI / 2.;
    let hip_angle = PI / 4.;
    let pelvis_angle = 0. + 0.1;
    let base_angle = 0. + PI / 2.;
    let q_init = vec![
        JointPosition::Pose(Pose {
            rotation: UnitQuaternion::identity(),
            translation: zero(),
        }),
        // JointPosition::None,
        JointPosition::Float(calf_angle),
        JointPosition::Float(thigh_angle),
        JointPosition::Float(hip_angle),
        JointPosition::Float(pelvis_angle),
        JointPosition::Float(base_angle),
    ];

    state.update_q(&q_init);

    flog!("center of mass: {}", state.center_of_mass());

    InterfaceMechanismState { inner: state }
}
