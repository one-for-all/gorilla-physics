use na::vector;

use crate::{
    joint::JointTorque, mechanism::MechanismState, spatial::spatial_vector::SpatialVector,
};

use super::{ControlInput, Controller};

pub struct NavbotController {}

impl Controller for NavbotController {
    fn control(
        &mut self,
        state: &mut MechanismState,
        _input: Option<&ControlInput>,
    ) -> Vec<JointTorque> {
        let q_target = 1.0;
        let kp = 1.0;
        let kd = 0.1;
        let tau = kp * (q_target - state.q[0].pose().translation.x)
            + kd * (-state.v[0].spatial().angular.y);

        let max_tau = 0.001;
        let tau = tau.signum() * tau.abs().min(max_tau);

        vec![JointTorque::Spatial(SpatialVector {
            angular: vector![0., tau, 0.],
            linear: vector![0., 0., 0.],
        })]
    }
}
