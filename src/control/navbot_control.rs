use crate::{
    flog, joint::JointTorque, mechanism::MechanismState, spatial::spatial_vector::SpatialVector,
    types::Float, PI,
};

use super::{ControlInput, Controller};

struct PIModule {
    pub P: Float,
    pub I: Float,
    pub dt: Float,

    integral_prev: Float,
    error_prev: Float,
}

impl PIModule {
    pub fn new(P: Float, I: Float, dt: Float) -> Self {
        PIModule {
            P,
            I,
            dt,
            integral_prev: 0.,
            error_prev: 0.,
        }
    }

    pub fn compute(&mut self, error: Float) -> Float {
        let proportional = self.P * error;
        let integral = self.integral_prev + self.I * self.dt * 0.5 * (self.error_prev + error);
        // TODO: limit the integral

        self.integral_prev = integral;
        self.error_prev = error;

        proportional + integral
    }
}

pub struct NavbotController {
    pi_angle: PIModule,
    pi_gyro: PIModule,
    pi_distance: PIModule,
    pi_speed: PIModule,
    pi_u: PIModule,

    angle_zeropoint: Float,
}

impl NavbotController {
    pub fn new(dt: Float) -> Self {
        NavbotController {
            pi_angle: PIModule::new(1., 0., dt),
            pi_gyro: PIModule::new(0.06, 0., dt),
            pi_distance: PIModule::new(0.5, 0., dt),
            pi_speed: PIModule::new(0.7, 0., dt),
            pi_u: PIModule::new(1., 15., dt),

            angle_zeropoint: 1.7, // -2.25,
        }
    }
}

impl Controller for NavbotController {
    fn control(
        &mut self,
        state: &mut MechanismState,
        _input: Option<&ControlInput>,
    ) -> Vec<JointTorque> {
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

        // Compute wheel control
        let body_pose = state.q[0].pose();
        let body_vel = state.v[0].spatial();

        let rad2degree = 180.0 / PI;
        let lqr_angle = body_pose.rotation.euler_angles().0 * rad2degree;
        let lqr_gyro = body_vel.angular[0] * rad2degree;

        let angle_control = self.pi_angle.compute(lqr_angle - self.angle_zeropoint);
        let gyro_control = self.pi_gyro.compute(lqr_gyro);

        let r = 0.037 / 2.0; // wheel radius
        let left_wheel_angle = state.q[4].float();
        let right_wheel_angle = -state.q[8].float();
        let left_wheel_velocity = state.v[4].float();
        let right_wheel_velocity = -state.v[8].float();
        let lqr_distance = (left_wheel_angle + right_wheel_angle) / 2.0; // distance proportional to wheel rotation angle
        let lqr_speed = (left_wheel_velocity + right_wheel_velocity) / 2.0; // velocity proportional to wheel rotational velocity

        flog!("distance: {}", lqr_distance * r);
        flog!("speed: {}", lqr_speed * r);

        let distance_control = self.pi_distance.compute(lqr_distance);
        let speed_control = self.pi_speed.compute(lqr_speed - 0.37 / r);
        let distance_control = 0.0;

        let control_voltage = angle_control + gyro_control + distance_control + speed_control;
        let control_voltage = self.pi_u.compute(control_voltage);
        self.angle_zeropoint -= 0.002 * distance_control; // critical. This finds the balance angle of the robot
        flog!("angle zeropoint: {}", self.angle_zeropoint);

        // rough wheel motor voltage to torque ration
        let voltage_to_torque = 0.0049; // https://item.taobao.com/item.htm?_u=k2st8hc2dcef&id=556445606114
        let wheel_tau = control_voltage * voltage_to_torque;

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
