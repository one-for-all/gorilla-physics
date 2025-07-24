use std::f64::INFINITY;

use crate::{
    control::servo::ServoMotor, flog, joint::JointTorque, mechanism::MechanismState,
    spatial::spatial_vector::SpatialVector, types::Float, PI,
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
    pub limit: Float,
    pub dt: Float,

    integral_prev: Float,
    error_prev: Float,
}

impl PIModule {
    pub fn new(P: Float, I: Float, limit: Float, dt: Float) -> Self {
        PIModule {
            P,
            I,
            limit,
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

        (proportional + integral).clamp(-self.limit, self.limit)
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
    pi_roll_angle: PIModule,
    lpf_roll: LowPassFilter,

    pi_lqr_u: PIModule,

    pi_zeropoint: PIModule,
    lpf_zeropoint: LowPassFilter,

    // pi_leg_left_q: PIModule,
    // pi_leg_right_q: PIModule,
    servo_left: ServoMotor,
    servo_right: ServoMotor,

    lpf_joyy: LowPassFilter,

    angle_zeropoint: Float,
    distance_zeropoint: Float,

    jump_flag: usize,
    move_stop_flag: bool,

    joyy_last: Float,
    dir_last: Float,
}

impl NavbotController {
    pub fn new(dt: Float) -> Self {
        NavbotController {
            pi_angle: PIModule::new(1., 0., 8., dt),
            pi_gyro: PIModule::new(0.06, 0., 8., dt),
            pi_distance: PIModule::new(0.5, 0., 8., dt),
            pi_speed: PIModule::new(0.7, 0., 8., dt),

            pi_yaw_gyro: PIModule::new(0.04, 0., 8., dt),
            pi_lqr_u: PIModule::new(1., 15., 8., dt),
            pi_zeropoint: PIModule::new(0.002, 0., 4., dt),

            pi_roll_angle: PIModule::new(8., 0., 450., dt),

            // pi_leg_left_q: PIModule::new(1., 10., 8., dt),
            // pi_leg_right_q: PIModule::new(1., 10., 8., dt),
            servo_left: ServoMotor::new(1., 10., 0.1, dt),
            servo_right: ServoMotor::new(1., 10., 0.1, dt),

            lpf_joyy: LowPassFilter::new(0.2, dt),
            lpf_zeropoint: LowPassFilter::new(0.1, dt),
            lpf_roll: LowPassFilter::new(0.3, dt),

            angle_zeropoint: 4.3, // 1.60, // -2.25,
            distance_zeropoint: 0.,

            jump_flag: 0,
            move_stop_flag: false,

            joyy_last: 0.,
            dir_last: 0.,
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
        let mut dir = 0.;
        if let Some(input) = input {
            let input = &input.floats;
            joyy = input[1];
            dir = input[2];
        }

        // TODO: maybe should be (or) to indicate in-air if either wheel not touching
        let in_air = state.bodies[4]
            .collider
            .as_ref()
            .unwrap()
            .geometry
            .sphere()
            .contact_halfspace(&state.halfspaces[0])
            .is_none()
            || state.bodies[8]
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
        if self.joyy_last != 0.0 && joyy == 0.0 {
            // stop when joy value turns zero
            self.move_stop_flag = true;
        }
        if self.move_stop_flag == true && lqr_speed.abs() < 0.5 {
            self.distance_zeropoint = lqr_distance;
            self.move_stop_flag = false;
        }
        if lqr_speed.abs() > 15. {
            // stop when pushed at high speed
            self.distance_zeropoint = lqr_distance;
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

        if lqr_u.abs() < 5. && joyy == 0.0 && distance_control.abs() < 4.0 && self.jump_flag == 0 {
            lqr_u = self.pi_lqr_u.compute(lqr_u);
            self.angle_zeropoint -= self
                .pi_zeropoint
                .compute(self.lpf_zeropoint.compute(distance_control));
            // critical. This finds the balance angle of the robot
        } else {
            self.pi_lqr_u.error_prev = 0.0;
        }

        let q_left = state.q[1].float();
        let v_left = state.v[1].float();
        let q_right = state.q[5].float();
        let v_right = state.v[5].float();

        self.servo_left.update(*q_left, *v_left);
        self.servo_right.update(*q_right, *v_right);

        // Jump
        let mut tau_left = 0.0;
        let mut tau_right = 0.0;
        if self.dir_last == 1.0 && dir == 0.0 && self.jump_flag == 0 {
            self.jump_flag = 1;
            self.servo_left.set(-0.2);
            self.servo_right.set(0.2);
        }
        if self.jump_flag > 0 {
            self.jump_flag += 1;
            // tau_left = -0.15;
            // tau_right = 0.15;
            if self.jump_flag > 50 {
                // Stabilize the leg angle
                // let kd = 0.2;
                // let q_target = 0.0;
                // tau_left = self.pi_leg_left_q.compute(-q_target - q_left) + kd * (0. - v_left);
                // tau_right = self.pi_leg_right_q.compute(q_target - q_right) + kd * (0. - v_right);

                self.servo_left.set(0.0);
                self.servo_right.set(0.0);
            }
            if self.jump_flag > 400 {
                self.jump_flag = 0;
            }
        }

        let roll_angle = body_pose.rotation.euler_angles().1 * rad2degree;
        flog!("roll: {}", roll_angle);
        flog!("lqr angle: {}", lqr_angle);
        flog!("angle zeropoint: {}", self.angle_zeropoint);
        flog!("distance: {}", lqr_distance - self.distance_zeropoint);
        flog!("=====");

        if self.jump_flag == 0 {
            // Stabilize the leg angle

            let leg_position_add = self.pi_roll_angle.compute(roll_angle);
            let leg_position_add = 0.01 * leg_position_add;

            let q_target = 0.0;
            let q_left_target = -(q_target - leg_position_add);
            let q_right_target = q_target + leg_position_add;

            // tau_left = self.pi_leg_left_q.compute(q_left_target - q_left) + kd * (0. - v_left);
            // tau_right = self.pi_leg_right_q.compute(q_right_target - q_right) + kd * (0. - v_right);

            self.servo_left.set(q_left_target);
            self.servo_right.set(q_right_target);
        }

        tau_left = self.servo_left.compute();
        tau_right = self.servo_right.compute();

        // yaw control
        let yaw_gyro = body_vel.angular.z * rad2degree;
        let yaw_gyro = self.pi_yaw_gyro.compute(yaw_gyro);
        let yaw_output = yaw_gyro;

        // rough wheel motor voltage to torque ration
        let voltage_to_torque = 0.0049; // https://item.taobao.com/item.htm?_u=k2st8hc2dcef&id=556445606114
        let left_wheel_tau = (lqr_u - yaw_output) / 2.0 * voltage_to_torque;
        let right_wheel_tau = -(lqr_u + yaw_output) / 2.0 * voltage_to_torque;

        // record motion input
        self.joyy_last = joyy;
        self.dir_last = dir;

        let max_leg_torque = 0.45;
        //flog!("left: {}", tau_left);
        //flog!("right: {}", tau_right);
        tau_left = tau_left.clamp(-max_leg_torque, max_leg_torque);
        tau_right = tau_right.clamp(-max_leg_torque, max_leg_torque);

        // Big TODO: model softness of the wheel rubber

        vec![
            // JointTorque::Float(0.),
            JointTorque::Spatial(SpatialVector::zero()),
            JointTorque::Float(tau_left),
            JointTorque::Float(0.),
            JointTorque::Float(0.),
            JointTorque::Float(left_wheel_tau),
            JointTorque::Float(tau_right),
            JointTorque::Float(0.),
            JointTorque::Float(0.),
            JointTorque::Float(right_wheel_tau),
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
