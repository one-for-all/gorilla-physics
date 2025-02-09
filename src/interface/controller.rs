use crate::{control::energy_control::HopperController, types::Float};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct InterfaceHopperController(pub(crate) HopperController);

#[wasm_bindgen]
pub fn createHopperController(
    h_setpoint: Float,
    body_leg_length: Float,
    leg_foot_length: Float,
) -> InterfaceHopperController {
    InterfaceHopperController(HopperController {
        k_spring: 200.0,
        h_setpoint,
        leg_length_setpoint: 0.0,
        v_vertical_prev: 0.0,
        body_leg_length,
        leg_foot_length,
    })
}
