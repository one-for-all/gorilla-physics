use na::vector;

use crate::{
    flog, joint::JointTorque, mechanism::MechanismState, spatial::spatial_vector::SpatialVector,
};

use super::{ControlInput, Controller};

pub struct NavbotController {}

impl Controller for NavbotController {
    fn control(
        &mut self,
        state: &mut MechanismState,
        _input: Option<&ControlInput>,
    ) -> Vec<JointTorque> {
        // let q_target = 1.0;
        // let kp = 1.0;
        // let kd = 0.1;
        // let tau = kp * (q_target - state.q[0].pose().translation.x)
        //     + kd * (-state.v[0].spatial().angular.y);

        // let max_tau = 0.001;
        // let tau = tau.signum() * tau.abs().min(max_tau);

        // vec![JointTorque::Spatial(SpatialVector {
        //     angular: vector![0., tau, 0.],
        //     linear: vector![0., 0., 0.],
        // })]
        //

        return vec![];

        vec![
            JointTorque::Float(0.),
            JointTorque::Float(0.),
            JointTorque::Float(0.),
            JointTorque::Float(0.),
        ]
    }
}

pub struct BalancingBotController {}

impl Controller for BalancingBotController {
    fn control(
        &mut self,
        state: &mut MechanismState,
        _input: Option<&ControlInput>,
    ) -> Vec<JointTorque> {
        let mut tau_vec = vec![JointTorque::Spatial(SpatialVector::zero())];

        let body_pose = state.q[0].pose();

        let kp_angular = 1.0;
        let kd_angular = 0.06;

        let angle = body_pose.rotation.euler_angles().0;
        let angular_vel = state.v[0].spatial().angular[0];

        let kp_linear = 0.5;
        let kd_linear = 0.7;

        let r = 0.02;
        let distance = state.q[1].float() * r; // distance calculated from wheel rotation angle
        let velocity = state.v[1].float() * r; // velocity calculated from wheel rotational velocity

        flog!(
            "angle: {:?}\nangular vel: {:?}\ndistance: {:?}\nvel: {:?}",
            angle,
            angular_vel,
            distance,
            velocity
        );

        let tau = kp_angular * angle + kd_angular * angular_vel
            - kp_linear * (0.1 - distance)
            - kd_linear * (0. - velocity);

        let max_tau = 0.01;
        let tau = tau.signum() * tau.abs().min(max_tau);
        let tau = JointTorque::Float(tau);
        flog!("tau: {:?}", tau);
        tau_vec.push(tau.clone());
        tau_vec.push(tau);

        tau_vec
    }
}
