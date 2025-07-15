use crate::{
    control::{ControlInput, Controller},
    joint::JointTorque,
    mechanism::MechanismState,
    PI,
};

pub struct FourBarLinkageController {}

impl Controller for FourBarLinkageController {
    fn control(
        &mut self,
        state: &mut MechanismState,
        _input: Option<&ControlInput>,
    ) -> Vec<JointTorque> {
        let q = state.q[1].float();
        let v = state.v[1].float();
        let tau = 10.0 * (PI / 4.0 - q) + 1.0 * (0. - v);
        vec![
            JointTorque::Float(0.),
            JointTorque::Float(tau),
            JointTorque::Float(0.),
        ]
    }
}

pub struct FourBarLinkageWithBaseController {}

impl Controller for FourBarLinkageWithBaseController {
    fn control(
        &mut self,
        state: &mut MechanismState,
        _input: Option<&ControlInput>,
    ) -> Vec<JointTorque> {
        let q = state.q[1].float();
        let v = state.v[1].float();
        let tau = 10.0 * (-PI / 2.0 - q) + 1.0 * (0. - v);
        vec![
            JointTorque::Float(0.),
            JointTorque::Float(tau),
            JointTorque::Float(0.),
            JointTorque::Float(0.),
        ]
    }
}
