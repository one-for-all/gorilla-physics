use crate::{
    control::energy_control::{Controller, Hopper1DController, Hopper2DController},
    joint::JointTorque,
    mechanism::MechanismState,
    types::Float,
};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct InterfaceController {
    pub(crate) inner: Box<dyn Controller>,
}

#[wasm_bindgen]
pub fn createHopper1DController(
    h_setpoint: Float,
    body_leg_length: Float,
    leg_foot_length: Float,
) -> InterfaceController {
    let inner: Box<dyn Controller> = Box::new(Hopper1DController {
        k_spring: 200.0,
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
) -> InterfaceController {
    let inner: Box<dyn Controller> = Box::new(Hopper2DController::new(
        h_setpoint,
        body_hip_length,
        hip_leg_length,
        leg_foot_length,
    ));
    InterfaceController { inner }
}

pub struct NullController {}

impl Controller for NullController {
    fn control(&mut self, _state: &MechanismState) -> Vec<JointTorque> {
        vec![]
    }
}

#[wasm_bindgen]
pub fn createNullController() -> InterfaceController {
    let inner: Box<dyn Controller> = Box::new(NullController {});
    InterfaceController { inner }
}
