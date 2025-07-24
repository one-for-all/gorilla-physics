use std::f64::INFINITY;

use crate::{
    flog, joint::JointTorque, mechanism::MechanismState, spatial::spatial_vector::SpatialVector,
    types::Float, PI,
};

use super::{ControlInput, Controller};

struct LowPassFilter {
    Tf: Float,     // low pass filter time constant
    y_prev: Float, // last output value
    dt: Float,
}

impl LowPassFilter {
    pub fn new(Tf: Float, dt: Float) -> Self {
        LowPassFilter {
            Tf,
            y_prev: INFINITY,
            dt,
        }
    }

    pub fn compute(&mut self, x: Float) -> Float {
        if self.y_prev.is_infinite() {
            self.y_prev = x;
            return x;
        }

        let alpha = self.Tf / (self.Tf + self.dt);
        let y = alpha * self.y_prev + (1.0 - alpha) * x;
        self.y_prev = y;
        y
    }
}

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

    pub fn reset(&mut self) {
        self.integral_prev = 0.;
        self.error_prev = 0.;
    }
}

pub struct NavbotController {
    pi_angle: PIModule,
    pi_gyro: PIModule,
    pi_distance: PIModule,
    pi_speed: PIModule,

    pi_yaw_gyro: PIModule,

    pi_lqr_u: PIModule,

    pi_zeropoint: PIModule,
    lpf_zeropoint: LowPassFilter,

    pi_leg_left_q: PIModule,
    pi_leg_right_q: PIModule,

    lpf_joyy: LowPassFilter,

    angle_zeropoint: Float,
    distance_zeropoint: Float,

    jump_flag: usize,
}

impl NavbotController {
    pub fn new(dt: Float) -> Self {
        NavbotController {
            pi_angle: PIModule::new(1., 0., dt),
            pi_gyro: PIModule::new(0.06, 0., dt),
            pi_distance: PIModule::new(0.5, 0., dt),
            pi_speed: PIModule::new(0.7, 0., dt),

            pi_yaw_gyro: PIModule::new(0.04, 0., dt),

            pi_lqr_u: PIModule::new(1., 15., dt),

            pi_zeropoint: PIModule::new(0.002, 0., dt),
            lpf_zeropoint: LowPassFilter::new(0.1, dt),

            pi_leg_left_q: PIModule::new(1., 15., dt),
            pi_leg_right_q: PIModule::new(1., 15., dt),

            lpf_joyy: LowPassFilter::new(0.2, dt),

            angle_zeropoint: 1.60, // -2.25,
            distance_zeropoint: 0.,

            jump_flag: 0,
        }
    }
}

impl Controller for NavbotController {
    fn control(
        &mut self,
        state: &mut MechanismState,
        input: Option<&ControlInput>,
    ) -> Vec<JointTorque> {
        let mut joyy = 0.;
        if let Some(input) = input {
            let input = &input.floats;
            joyy = input[1];
        }

        let in_air = state.bodies[4]
            .collider
            .as_ref()
            .unwrap()
            .geometry
            .sphere()
            .contact_halfspace(&state.halfspaces[0])
            .is_none();

        // Relevant states
        let body_pose = state.q[0].pose();
        let body_vel = state.v[0].spatial();

        let left_wheel_angle = state.q[4].float();
        let right_wheel_angle = -state.q[8].float();
        let left_wheel_velocity = state.v[4].float();
        let right_wheel_velocity = -state.v[8].float();

        // LQR balance control
        let lqr_distance = (left_wheel_angle + right_wheel_angle) / 2.0; // distance proportional to wheel rotation angle
        let lqr_speed = (left_wheel_velocity + right_wheel_velocity) / 2.0; // velocity proportional to wheel rotational velocity

        let rad2degree = 180.0 / PI;
        let lqr_angle = body_pose.rotation.euler_angles().0 * rad2degree;
        let lqr_gyro = body_vel.angular[0] * rad2degree;

        let angle_control = self.pi_angle.compute(lqr_angle - self.angle_zeropoint);
        let gyro_control = self.pi_gyro.compute(lqr_gyro);

        // TODO: motion details
        if joyy != 0.0 {
            self.distance_zeropoint = lqr_distance;
            self.pi_lqr_u.error_prev = 0.0;
        }

        // linear motion control
        // let r = 0.037 / 2.0; // wheel radius
        let distance_control = self
            .pi_distance
            .compute(lqr_distance - self.distance_zeropoint);
        let speed_control = self
            .pi_speed
            .compute(lqr_speed - 0.1 * self.lpf_joyy.compute(joyy));

        let mut lqr_u;
        if in_air {
            self.distance_zeropoint = lqr_distance;
            lqr_u = angle_control + gyro_control;
            self.pi_lqr_u.error_prev = 0.0;
        } else {
            lqr_u = angle_control + gyro_control + distance_control + speed_control;
        }

        if !in_air && joyy == 0.0 {
            lqr_u = self.pi_lqr_u.compute(lqr_u);
            self.angle_zeropoint -= self
                .pi_zeropoint
                .compute(self.lpf_zeropoint.compute(speed_control)); // critical. This finds the balance angle of the robot
        } else {
            self.pi_lqr_u.error_prev = 0.0;
        }

        let q_left = state.q[1].float();
        let v_left = state.v[1].float();
        let q_right = state.q[5].float();
        let v_right = state.v[5].float();

        // Jump
        let mut tau_left = 0.0;
        let mut tau_right = 0.0;
        // if let Some(input) = _input {
        //     if input.floats[2] > 0. && self.jump_flag == 0 {
        //         self.jump_flag = 1;
        //         self.pi_leg_left_q.reset();
        //         self.pi_leg_right_q.reset();
        //     }
        // }
        // if self.jump_flag > 0 {
        //     self.jump_flag += 1;

        //     let jump_time = 50;
        //     if self.jump_flag < jump_time {
        //         tau_left = -0.15;
        //         tau_right = 0.15;
        //     } else {
        //         self.jump_flag = 0;
        //     }
        // }

        if self.jump_flag == 0 {
            // Stabilize the leg angle
            let kd = 0.1;
            let q_target = 0.0;
            tau_left = self.pi_leg_left_q.compute(-q_target - q_left) + kd * (0. - v_left);
            tau_right = self.pi_leg_right_q.compute(q_target - q_right) + kd * (0. - v_right);
        }

        // rough wheel motor voltage to torque ration
        let voltage_to_torque = 0.0049; // https://item.taobao.com/item.htm?_u=k2st8hc2dcef&id=556445606114
        let wheel_tau = lqr_u * voltage_to_torque;

        let yaw_gyro = body_vel.angular.z * rad2degree * voltage_to_torque;
        let yaw_gyro = self.pi_yaw_gyro.compute(yaw_gyro);

        vec![
            // JointTorque::Float(0.),
            JointTorque::Spatial(SpatialVector::zero()),
            JointTorque::Float(tau_left),
            JointTorque::Float(0.),
            JointTorque::Float(0.),
            JointTorque::Float((wheel_tau - yaw_gyro) / 2.0),
            JointTorque::Float(tau_right),
            JointTorque::Float(0.),
            JointTorque::Float(0.),
            JointTorque::Float((-wheel_tau - yaw_gyro) / 2.0),
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
