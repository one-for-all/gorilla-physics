use crate::{
    flog, joint::JointTorque, mechanism::MechanismState, spatial::spatial_vector::SpatialVector, PI,
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

        // No control when in the air
        if state.bodies[4]
            .collider
            .as_ref()
            .unwrap()
            .geometry
            .sphere()
            .contact_halfspace(&state.halfspaces[0])
            .is_none()
        {
            return vec![];
        }

        // Stabilize the leg angle
        let q_left = state.q[1].float();
        let v_left = state.v[1].float();
        let q_right = state.q[5].float();
        let v_right = state.v[5].float();

        let kp = 1.0;
        let kd = 0.1;
        let tau_left = kp * (-0.1 - q_left) + kd * (0. - v_left);
        let tau_right = kp * (0.1 - q_right) + kd * (0. - v_right);

        flog!("q_left: {}", q_left);
        flog!("q_right: {}", q_right);

        let body_pose = state.q[0].pose();

        let kp_angular = 0.3;
        let kd_angular = 0.06;

        let angle = body_pose.rotation.euler_angles().0;
        flog!("angle: {}", angle);
        // let angle = angle.signum() * (angle.abs().max(0.15) - 0.15);
        let angular_vel = state.v[0].spatial().angular[0];

        let kp_linear = 3.5;
        let kd_linear = 0.9;

        let r = 0.037 / 2.0; // wheel radius
        let left_wheel_angle = state.q[4].float();
        let right_wheel_angle = -state.q[8].float();
        let left_wheel_velocity = state.v[4].float();
        let right_wheel_velocity = -state.v[8].float();
        let lqr_distance = (left_wheel_angle + right_wheel_angle) / 2.0; // distance proportional to wheel rotation angle
        let lqr_speed = (left_wheel_velocity + right_wheel_velocity) / 2.0; // velocity proportional to wheel rotational velocity

        let lqr_distance = (left_wheel_angle + right_wheel_angle) / 2.0 * r; // distance proportional to wheel rotation angle
        let lqr_speed = (left_wheel_velocity + right_wheel_velocity) / 2.0 * r; // velocity proportional to wheel rotational velocity

        let vel_target = {
            if _input.unwrap().floats[1] > 0. {
                0.1
            } else if _input.unwrap().floats[1] < 0. {
                -0.1
            } else {
                0.0
            }
        };

        let wheel_tau = kp_angular * angle + kd_angular * angular_vel
            // - kp_linear * (0.05 - distance)
            - kd_linear * (vel_target - lqr_speed); // 0.2

        flog!("distance: {}", lqr_distance);
        flog!("velocity: {}", lqr_speed);
        flog!("wheel_tau: {}", wheel_tau);

        let max_tau = 0.02;
        let wheel_tau = wheel_tau.signum() * wheel_tau.abs().min(max_tau);

        vec![
            // JointTorque::Float(0.),
            JointTorque::Spatial(SpatialVector::zero()),
            JointTorque::Float(tau_left),
            JointTorque::Float(0.),
            JointTorque::Float(0.),
            JointTorque::Float(wheel_tau),
            JointTorque::Float(tau_right),
            JointTorque::Float(0.),
            JointTorque::Float(0.),
            JointTorque::Float(-wheel_tau),
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
