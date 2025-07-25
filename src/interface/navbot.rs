use futures::future::join_all;
use na::{vector, zero, UnitQuaternion};
use wasm_bindgen::prelude::*;
use web_sys::window;

use crate::{
    builders::navbot_builder::{
        build_balancing_bot, build_navbot, build_navbot_motor, NavbotMeshes,
    },
    collision::mesh::Mesh,
    flog,
    joint::{JointPosition, JointVelocity},
    spatial::{pose::Pose, spatial_vector::SpatialVector},
    types::Float,
};

use super::{util::read_web_file, InterfaceMechanismState};

#[wasm_bindgen]
pub async fn createNavbotMotor() -> InterfaceMechanismState {
    let buf = read_web_file("navbot/quanum_gimbal_motor_2208_base.obj").await;
    let mesh = Mesh::new_from_obj(&buf, false);

    let mut state = build_navbot_motor(mesh);

    let q_init = vec![JointPosition::Pose(Pose {
        rotation: UnitQuaternion::identity(),
        translation: vector![0., 0., 0.01385],
    })];
    state.update_q(&q_init);

    InterfaceMechanismState { inner: state }
}

#[wasm_bindgen]
pub async fn createNavbot() -> InterfaceMechanismState {
    let performance = window().unwrap().performance().unwrap();
    let start = performance.now();

    let mut navbot_meshes = NavbotMeshes::new();

    let file_paths = vec![
        "navbot/merged/base_visual.obj",
        "navbot/merged/leg_left_visual.obj",
        "navbot/merged/foot_left_visual.obj",
        "navbot/merged/link_visual.obj",
        "navbot/merged/wheel_motor_visual.obj",
        "navbot/merged/leg_right_visual.obj",
        "navbot/merged/foot_right_visual.obj",
        "navbot/merged/link_2_visual.obj",
        "navbot/merged/wheel_motor_2_visual.obj",
    ];

    let fetches = file_paths.iter().map(|path| read_web_file(path));
    let buffers: Vec<String> = join_all(fetches).await;
    for (i, buf) in buffers.iter().enumerate() {
        let mesh = Some(Mesh::new_from_obj(buf, false));
        match i {
            0 => navbot_meshes.base = mesh,
            1 => navbot_meshes.leg_left = mesh,
            2 => navbot_meshes.foot_left = mesh,
            3 => navbot_meshes.link_left = mesh,
            4 => navbot_meshes.wheel_left = mesh,
            5 => navbot_meshes.leg_right = mesh,
            6 => navbot_meshes.foot_right = mesh,
            7 => navbot_meshes.link_right = mesh,
            8 => navbot_meshes.wheel_right = mesh,
            _ => {}
        }
    }

    let end = performance.now();
    flog!("loading mesh took {:.3} s", (end - start) / 1000.); // currently takes ~2 seconds

    let mut state = build_navbot(navbot_meshes);

    // let q_init = vec![JointPosition::Float(0.)];
    // state.update_q(&q_init);
    //
    // state.set_joint_v(1, JointVelocity::Float(0.1));
    state.set_joint_q(
        1,
        JointPosition::Pose(Pose {
            rotation: UnitQuaternion::from_euler_angles(0., 0., 0.),
            translation: vector![0., 0., 0.075],
        }),
    );
    state.set_joint_v(
        1,
        JointVelocity::Spatial(SpatialVector {
            linear: zero(),
            angular: vector![0., 0., 0.],
        }),
    );

    InterfaceMechanismState { inner: state }
}

#[wasm_bindgen]
pub async fn createBalancingBot() -> InterfaceMechanismState {
    let mut state = build_balancing_bot();

    let h_body = 0.025;
    let h_offset = h_body * 2.0;
    let r_wheel = 0.02;
    let q_body_init = JointPosition::Pose(Pose {
        // rotation: UnitQuaternion::identity(),
        rotation: UnitQuaternion::from_euler_angles(0., 0., 0.),
        translation: vector![0., 0.1, h_offset + r_wheel],
    });
    state.set_joint_q(1, q_body_init);

    InterfaceMechanismState { inner: state }
}
