use crate::{joint::JointTorque, mechanism::MechanismState, spatial_vector::SpatialVector, PI};

use super::energy_control::Controller;

pub struct QuadrupedStandingController {}

impl Controller for QuadrupedStandingController {
    /// Control the quadruped to stand still
    fn control(&mut self, state: &mut MechanismState) -> Vec<JointTorque> {
        let mut tau = vec![JointTorque::Spatial(SpatialVector::zero())]; // first floating joint unactuated

        let kp = 100.0;
        let kd = 30.0;

        let mut i = 1;
        while i < 9 {
            let q_hip = state.q[i].float();
            let q_knee = state.q[i + 1].float();
            let v_hip = state.v[i].float();
            let v_knee = state.v[i + 1].float();

            let tau_hip = kp * (-PI / 4.0 - q_hip) + kd * -v_hip;
            tau.push(JointTorque::Float(tau_hip));

            let tau_knee = kp * (PI / 2.0 - q_knee) + kd * -v_knee;
            tau.push(JointTorque::Float(tau_knee));

            i += 2;
        }

        tau
    }
}
