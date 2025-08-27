use crate::{
    control::{
        biped_control::BipedController,
        energy_control::{Hopper1DController, Hopper2DController},
        gripper_control::GripperManualController,
        kinematic_loop::{FourBarLinkageController, FourBarLinkageWithBaseController},
        leg_control::{LegController, LegFromFootController},
        navbot_control::{BalancingBotController, NavbotController},
        pusher_control::PusherController,
        quadruped_control::{QuadrupedStandingController, QuadrupedTrottingController},
        so101_control::SO101PositionController,
        Controller, NullController,
    },
    types::Float,
};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct InterfaceController {
    pub(crate) inner: Box<dyn Controller>,
}

#[wasm_bindgen]
pub fn createQuadrupedStandingController() -> InterfaceController {
    let inner: Box<dyn Controller> = Box::new(QuadrupedStandingController {});
    InterfaceController { inner }
}

#[wasm_bindgen]
pub fn createQuadrupedTrottingController(
    dt: Float,
    target_x: Float,
    default_foot_z: Float,
) -> InterfaceController {
    let inner: Box<dyn Controller> = Box::new(QuadrupedTrottingController::new(
        dt,
        target_x,
        default_foot_z,
    ));
    InterfaceController { inner }
}

#[wasm_bindgen]
pub fn createHopper1DController(
    h_setpoint: Float,
    body_leg_length: Float,
    leg_foot_length: Float,
) -> InterfaceController {
    let inner: Box<dyn Controller> = Box::new(Hopper1DController {
        k_spring: 1e3,
        h_setpoint,
        leg_length_setpoint: 0.0,
        v_vertical_prev: 0.0,
        body_leg_length,
        leg_foot_length,
    });
    InterfaceController { inner }
}

#[wasm_bindgen]
pub fn createHopper2DController(
    h_setpoint: Float,
    body_hip_length: Float,
    hip_leg_length: Float,
    leg_foot_length: Float,
    l_leg: Float,
    k_spring: Float,
) -> InterfaceController {
    let inner: Box<dyn Controller> = Box::new(Hopper2DController::new(
        h_setpoint,
        body_hip_length,
        hip_leg_length,
        leg_foot_length,
        l_leg,
        k_spring,
    ));
    InterfaceController { inner }
}

#[wasm_bindgen]
pub fn createPusherController() -> InterfaceController {
    let inner: Box<dyn Controller> = Box::new(PusherController {});
    InterfaceController { inner }
}

#[wasm_bindgen]
pub fn createGripperManualController() -> InterfaceController {
    let inner: Box<dyn Controller> = Box::new(GripperManualController {});
    InterfaceController { inner }
}

#[wasm_bindgen]
pub fn createSO101PositionController() -> InterfaceController {
    let inner: Box<dyn Controller> = Box::new(SO101PositionController {});
    InterfaceController { inner }
}

#[wasm_bindgen]
pub fn createNavbotController(dt: Float) -> InterfaceController {
    let inner: Box<dyn Controller> = Box::new(NavbotController::new(dt));
    InterfaceController { inner }
}

#[wasm_bindgen]
pub fn createBalancingBotController() -> InterfaceController {
    let inner: Box<dyn Controller> = Box::new(BalancingBotController {});
    InterfaceController { inner }
}

#[wasm_bindgen]
pub fn createFourBarLinkageController() -> InterfaceController {
    let inner: Box<dyn Controller> = Box::new(FourBarLinkageController {});
    InterfaceController { inner }
}

#[wasm_bindgen]
pub fn createFourBarLinkageWithBaseController() -> InterfaceController {
    let inner: Box<dyn Controller> = Box::new(FourBarLinkageWithBaseController {});
    InterfaceController { inner }
}

#[wasm_bindgen]
pub fn createLegFromFootController() -> InterfaceController {
    let inner: Box<dyn Controller> = Box::new(LegFromFootController {});
    InterfaceController { inner }
}

#[wasm_bindgen]
pub fn createLegController() -> InterfaceController {
    let inner: Box<dyn Controller> = Box::new(LegController {});
    InterfaceController { inner }
}

#[wasm_bindgen]
pub fn createBipedController() -> InterfaceController {
    let inner: Box<dyn Controller> = Box::new(BipedController {});
    InterfaceController { inner }
}

#[wasm_bindgen]
pub fn createNullController() -> InterfaceController {
    let inner: Box<dyn Controller> = Box::new(NullController {});
    InterfaceController { inner }
}
