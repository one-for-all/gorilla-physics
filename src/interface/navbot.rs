use futures::future::join_all;
use na::{vector, UnitQuaternion};
use wasm_bindgen::prelude::*;
use web_sys::window;

use crate::{
    builders::navbot::{build_balancing_bot, build_navbot, build_navbot_motor, NavbotMeshes},
    collision::mesh::Mesh,
    flog,
    joint::JointPosition,
    spatial::pose::Pose,
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
        "navbot/esp32pcb.obj",
        "navbot/top_plate.obj",
        "navbot/side_plate_left.obj",
        "navbot/side_plate_right.obj",
        "navbot/leg_inner_left.obj",
        "navbot/leg_outer_left.obj",
        "navbot/pin.obj",
        "navbot/foot_motor_left.obj",
        "navbot/encoder.obj",
        "navbot/foot_plate_left.obj",
        "navbot/link_plate_left.obj",
    ];

    let fetches = file_paths.iter().map(|path| read_web_file(path));
    let buffers: Vec<String> = join_all(fetches).await;
    for (i, buf) in buffers.iter().enumerate() {
        let mesh = Some(Mesh::new_from_obj(buf, false));
        match i {
            0 => navbot_meshes.esp32pcb = mesh,
            1 => navbot_meshes.top_plate = mesh,
            2 => navbot_meshes.side_plate_left = mesh,
            3 => navbot_meshes.side_plate_right = mesh,
            4 => navbot_meshes.leg_inner_left = mesh,
            5 => navbot_meshes.leg_outer_left = mesh,
            6 => navbot_meshes.foot_pin_left = mesh,
            7 => navbot_meshes.foot_motor_left = mesh,
            8 => navbot_meshes.foot_encoder_left = mesh,
            9 => navbot_meshes.foot_plate_left = mesh,
            10 => navbot_meshes.link_plate_left = mesh,
            _ => {}
        }
    }

    let end = performance.now();
    flog!("loading mesh took {:.3} s", (end - start) / 1000.); // currently takes ~2 seconds

    let state = build_navbot(navbot_meshes);

    // let q_init = vec![JointPosition::Float(0.)];
    // state.update_q(&q_init);
    //
    // state.set_joint_v(1, JointVelocity::Float(0.1));

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
