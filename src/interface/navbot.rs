use na::{vector, UnitQuaternion};
use wasm_bindgen::prelude::wasm_bindgen;

use crate::{
    builders::navbot::{build_balancing_bot, build_navbot, build_navbot_motor, NavbotMeshes},
    collision::mesh::Mesh,
    joint::JointPosition,
    spatial::pose::Pose,
};

use super::{util::read_web_file, InterfaceMechanismState};

#[wasm_bindgen]
pub async fn createNavbotMotor() -> InterfaceMechanismState {
    let buf = read_web_file("navbot/quanum_gimbal_motor_2208_base.obj").await;
    let mesh = Mesh::new_from_obj(&buf);

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
    let mut navbot_meshes = NavbotMeshes::new();
    let buf = read_web_file("navbot/esp32pcb.obj").await;
    let mesh = Mesh::new_from_obj(&buf);
    navbot_meshes.esp32pcb = Some(mesh);

    let buf = read_web_file("navbot/top_plate.obj").await;
    let mesh = Mesh::new_from_obj(&buf);
    navbot_meshes.top_plate = Some(mesh);

    let buf = read_web_file("navbot/side_plate_left.obj").await;
    let mesh = Mesh::new_from_obj(&buf);
    navbot_meshes.side_plate_left = Some(mesh);

    let buf = read_web_file("navbot/side_plate_right.obj").await;
    let mesh = Mesh::new_from_obj(&buf);
    navbot_meshes.side_plate_right = Some(mesh);

    // Leg left
    let buf = read_web_file("navbot/leg_inner_left.obj").await;
    let mesh = Mesh::new_from_obj(&buf);
    navbot_meshes.leg_inner_left = Some(mesh);

    let buf = read_web_file("navbot/leg_outer_left.obj").await;
    let mesh = Mesh::new_from_obj(&buf);
    navbot_meshes.leg_outer_left = Some(mesh);

    // Foot left
    let pin_buf = read_web_file("navbot/pin.obj").await;
    let pin_mesh = Mesh::new_from_obj(&pin_buf);
    navbot_meshes.foot_pin_left = Some(pin_mesh);

    let buf = read_web_file("navbot/foot_motor_left.obj").await;
    let mesh = Mesh::new_from_obj(&buf);
    navbot_meshes.foot_motor_left = Some(mesh);

    let encoder_buf = read_web_file("navbot/encoder.obj").await;
    let encoder_mesh = Mesh::new_from_obj(&encoder_buf);
    navbot_meshes.foot_encoder_left = Some(encoder_mesh);

    let buf = read_web_file("navbot/foot_plate_left.obj").await;
    let mesh = Mesh::new_from_obj(&buf);
    navbot_meshes.foot_plate_left = Some(mesh);

    let buf = read_web_file("navbot/link_plate_left.obj").await;
    let mesh = Mesh::new_from_obj(&buf);
    navbot_meshes.link_plate_left = Some(mesh);

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
