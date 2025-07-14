use itertools::izip;

use crate::{
    joint::{JointPosition, JointTorque},
    mechanism::MechanismState,
};

use super::{ControlInput, Controller};

pub struct SO101PositionController {}

impl Controller for SO101PositionController {
    fn control(
        &mut self,
        state: &mut MechanismState,
        _input: Option<&ControlInput>,
    ) -> Vec<JointTorque> {
        let mut tau_vec = vec![];
        let kp = 100e1;
        let kd = 1e-1;
        for (q, v) in izip!(state.q.iter(), state.v.iter()) {
            let tau = if matches!(q, JointPosition::None) {
                JointTorque::None
            } else {
                let t = kp * (0.0 - q.float()) - kd * v.float();

                JointTorque::Float(t.signum() * t.abs().min(10.0))
            };
            tau_vec.push(tau);
        }

        tau_vec
    }
}
