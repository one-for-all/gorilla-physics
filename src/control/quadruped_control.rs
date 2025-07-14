use na::{vector, ComplexField, Vector3, Vector4};

use crate::{
    joint::JointTorque, mechanism::MechanismState, spatial::spatial_vector::SpatialVector,
    types::Float, PI,
};

use super::{ControlInput, Controller};

pub struct QuadrupedTrottingController {
    ticks: usize,
    contact_phases: Vector4<Vector4<bool>>,
    dt: Float,           // time interval of each tick
    overlap_time: Float, // duration of the phase where all four feet are on the ground
    swing_time: Float,   // duration of the phase where only two feet are on the ground
    z_clearance: Float,
    default_foot_z: Float,
    default_stance: Vec<Vector3<Float>>,

    foot_locations: Vec<Vector3<Float>>,
    vx: Float,
    target_x: Float,

    l_leg: Float, // Leg length
}

impl Controller for QuadrupedTrottingController {
    /// Controller for trotting forward
    fn control(
        &mut self,
        state: &mut MechanismState,
        _input: Option<&ControlInput>,
    ) -> Vec<JointTorque> {
        let x = state.q[0].pose().translation.x;
        let dx = x - self.target_x;
        let max_vx = 1.0;
        self.vx = -dx.signum() * dx.abs().scale(10.0).min(max_vx);

        let mut tau = vec![JointTorque::Spatial(SpatialVector::zero())];

        let foot_locations = self.step_gait();
        self.foot_locations = foot_locations.clone();

        let target_joint_angles =
            QuadrupedTrottingController::inverse_kinematics(&foot_locations, self.l_leg);

        self.ticks += 1;

        let kp = 150.0;
        let kd = 10.0;
        for (leg_index, angles) in target_joint_angles.iter().enumerate() {
            let hip_angle = angles[0];
            let knee_angle = angles[1];

            let hip_index = leg_index * 2 + 1;
            let knee_index = hip_index + 1;

            let q_hip = state.q[hip_index].float();
            let q_knee = state.q[knee_index].float();
            let v_hip = state.v[hip_index].float();
            let v_knee = state.v[knee_index].float();

            let tau_hip = kp * (hip_angle - q_hip) + kd * -v_hip;
            tau.push(JointTorque::Float(tau_hip));

            let tau_knee = kp * (knee_angle - q_knee) + kd * -v_knee;
            tau.push(JointTorque::Float(tau_knee));
        }

        tau
    }
}

impl QuadrupedTrottingController {
    pub fn new(dt: Float, target_x: Float, default_foot_z: Float) -> QuadrupedTrottingController {
        // contact/no-contact modes for fr, fl, br, bl legs
        let contact_phases = vector![
            vector![true, true, true, true],
            vector![false, true, true, false],
            vector![true, true, true, true],
            vector![true, false, false, true],
        ];

        // default foot positions relative to leg origin
        let default_foot_stance = vec![
            vector![0., 0., default_foot_z],
            vector![0., 0., default_foot_z],
            vector![0., 0., default_foot_z],
            vector![0., 0., default_foot_z],
        ];

        QuadrupedTrottingController {
            ticks: 0,
            contact_phases,
            dt,
            overlap_time: 0.1,
            swing_time: 0.15,
            z_clearance: 0.25,
            default_foot_z,
            default_stance: default_foot_stance.clone(),
            foot_locations: default_foot_stance.clone(),
            vx: 0.0,
            target_x,
            l_leg: 1.0,
        }
    }

    /// Target foot locations at current tick
    fn step_gait(&self) -> Vec<Vector3<Float>> {
        let contact_modes = self.contact_modes(self.ticks);
        let mut foot_locations = vec![];
        for leg_index in 0..4 {
            let contact_mode = contact_modes[leg_index];
            let foot_location = match contact_mode {
                true => self.next_stance_foot_location(leg_index),
                false => self.next_swing_foot_location(self.subphase_ticks(self.ticks), leg_index),
            };
            foot_locations.push(foot_location);
        }

        foot_locations
    }

    /// target foot location in stance phase
    fn next_stance_foot_location(&self, leg_index: usize) -> Vector3<Float> {
        let foot_location = self.foot_locations[leg_index];
        let z = foot_location[2];
        let h = self.default_foot_z;
        let v_xy = vector![-self.vx, 0.0, 1.0 / 0.02 * (h - z)];
        let delta_p = v_xy * self.dt;

        foot_location + delta_p
    }

    /// Target foot location in swing phase
    fn next_swing_foot_location(&self, swing_ticks: usize, leg_index: usize) -> Vector3<Float> {
        let swing_proportion = swing_ticks as Float / self.swing_ticks() as Float;

        if swing_proportion < 0.0 || swing_proportion > 1.0 {
            panic!("swing proportion: {}", swing_proportion);
        }
        let foot_location = self.foot_locations[leg_index];

        // Compute target foot swing height
        let height_proportion = (swing_ticks + 1) as Float / self.swing_ticks() as Float;
        let swing_height = {
            if height_proportion < 0.5 {
                self.z_clearance * height_proportion / 0.5
            } else {
                self.z_clearance * (1.0 - (height_proportion - 0.5) / 0.5)
            }
        };
        let z_vector = vector![0., 0., swing_height + self.default_foot_z];

        // Raibert touchdown location
        let alpha = 0.5;
        let delta_px = alpha * self.stance_ticks() as Float * self.dt * self.vx;
        let delta_p = vector![delta_px, 0., 0.];
        let touchdown_location = self.default_stance[leg_index] + delta_p;

        let time_left = self.dt * self.swing_ticks() as Float * (1.0 - swing_proportion);
        let v =
            ((touchdown_location - foot_location) / time_left).component_mul(&vector![1., 1., 0.]);
        let delta_foot_location = v * self.dt;

        foot_location.component_mul(&vector![1., 1., 0.]) + z_vector + delta_foot_location
    }

    /// Given foot locations, compute joint angles
    pub fn inverse_kinematics(
        foot_locations: &Vec<Vector3<Float>>,
        l_leg: Float,
    ) -> Vec<Vec<Float>> {
        let mut out = vec![];
        for foot_vec in foot_locations {
            let x = foot_vec[0];
            let z = foot_vec[2];

            let l1 = l_leg / 2.0;
            let l2 = l_leg / 2.0;

            let x2z2 = x * x + z * z;
            let cos_theta2 = (x2z2 - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);
            let theta2 = cos_theta2.acos();

            let mut theta1 = x.atan2(-z) - (l2 * theta2.sin()).atan2(l1 + l2 * theta2.cos());

            // TODO: fix this hack. Corner case when foot location is [0, 0, 0]
            if theta1 > 0.0 {
                theta1 -= PI;
            }

            out.push(vec![theta1, theta2]);
        }

        out
    }

    /// Given joint angles, compute foot locations
    pub fn forward_kinematics(joint_angles: &Vec<Vec<Float>>, l_leg: Float) -> Vec<Vector3<Float>> {
        let mut out = vec![];
        for angles in joint_angles {
            let theta1 = angles[0];
            let theta2 = angles[1];

            let l1 = l_leg / 2.0;
            let l2 = l_leg / 2.0;
            let x = -l1 * (-theta1).sin() + l2 * (theta2 + theta1).sin();
            let z = -l1 * (-theta1).cos() - l2 * (theta2 + theta1).cos();

            out.push(vector![x, 0., z]);
        }
        out
    }

    /// Return which feet should be in contact at the given number of ticks
    fn contact_modes(&self, ticks: usize) -> Vector4<bool> {
        let index = self.phase_index(ticks);
        self.contact_phases[index].clone()
    }

    /// Calculates which part of the gait cycle the robot should be in given
    /// the time in ticks
    fn phase_index(&self, ticks: usize) -> usize {
        let phase_time = ticks % self.period_ticks();
        let mut phase_sum = 0;
        for i in 0..4 {
            phase_sum += self.phase_ticks_vec()[i];
            if phase_time < phase_sum {
                return i;
            }
        }
        panic!("should not reach this");
    }

    /// Calculates the number of ticks since the start of the current phase
    fn subphase_ticks(&self, ticks: usize) -> usize {
        let phase_time = ticks % self.period_ticks();

        let mut phase_sum = 0;
        let subphase_tick_number;
        for i in 0..4 {
            phase_sum += self.phase_ticks_vec()[i];
            if phase_time < phase_sum {
                subphase_tick_number = phase_time + self.phase_ticks_vec()[i] - phase_sum;
                return subphase_tick_number;
            }
        }
        panic!("should not reach this");
    }

    /// Number of ticks in overlap phase
    fn overlap_ticks(&self) -> usize {
        (self.overlap_time / self.dt) as usize
    }

    /// Number of ticks in swing phase
    fn swing_ticks(&self) -> usize {
        (self.swing_time / self.dt) as usize
    }

    /// Number of ticks in stance phase
    fn stance_ticks(&self) -> usize {
        2 * self.overlap_ticks() + self.swing_ticks()
    }

    /// Number of ticks in a motion period
    fn period_ticks(&self) -> usize {
        2 * self.overlap_ticks() + 2 * self.swing_ticks()
    }

    /// vector of ticks of each phase in a motion period
    fn phase_ticks_vec(&self) -> Vector4<usize> {
        let overlap_ticks = self.overlap_ticks();
        let swing_ticks = self.swing_ticks();
        vector![overlap_ticks, swing_ticks, overlap_ticks, swing_ticks]
    }
}

pub struct QuadrupedStandingController {}

impl Controller for QuadrupedStandingController {
    /// Control the quadruped to stand still
    fn control(
        &mut self,
        state: &mut MechanismState,
        _input: Option<&ControlInput>,
    ) -> Vec<JointTorque> {
        let mut tau = vec![JointTorque::Spatial(SpatialVector::zero())]; // first floating joint unactuated

        let kp = 150.0;
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

#[cfg(test)]
mod quadruped_control_tests {
    use na::UnitQuaternion;

    use crate::{
        assert_close, assert_vec_close, contact::HalfSpace, helpers::build_quadruped,
        integrators::Integrator, joint::JointPosition, simulate::step, spatial::pose::Pose,
    };

    use super::*;

    #[test]
    fn test_inverse_kinematics_1() {
        // Arrange
        let foot_locations = vec![vector![0., 0., 0.]];

        // Act
        let joint_angles = QuadrupedTrottingController::inverse_kinematics(&foot_locations, 1.0);

        // Assert
        assert_vec_close!(&joint_angles[0], vec![-PI / 2.0, PI], 1e-5);
    }

    #[test]
    fn test_inverse_kinematics_2() {
        // Arrange
        let l_leg = 1.0;
        let foot_locations = vec![vector![0., 0., -l_leg]];

        // Act
        let joint_angles = QuadrupedTrottingController::inverse_kinematics(&foot_locations, l_leg);

        // Assert
        assert_vec_close!(&joint_angles[0], vec![0.0, 0.0], 1e-5);
    }

    #[test]
    fn test_inverse_kinematics_3() {
        // Arrange
        let l_leg = 1.0;
        let l1 = l_leg / 2.0;
        let l2 = l_leg / 2.0;
        let x_target = -0.0;
        let z_target = -0.5;
        let foot_locations = vec![vector![x_target, 0., z_target]];

        // Act
        let joint_angles = QuadrupedTrottingController::inverse_kinematics(&foot_locations, l_leg);

        // Assert
        let theta1 = joint_angles[0][0];
        let theta2 = joint_angles[0][1];
        let x = -l1 * (-theta1).sin() + l2 * (theta2 + theta1).sin();
        let z = -l1 * (-theta1).cos() - l2 * (theta2 + theta1).cos();
        assert_close!(x, x_target, 1e-5);
        assert_close!(z, z_target, 1e-5);
    }

    #[test]
    fn test_inverse_kinematics_4() {
        // Arrange
        let l_leg = 1.0;
        let l1 = l_leg / 2.0;
        let l2 = l_leg / 2.0;
        let x_target = -0.1;
        let z_target = -0.5;
        let foot_locations = vec![vector![x_target, 0., z_target]];

        // Act
        let joint_angles = QuadrupedTrottingController::inverse_kinematics(&foot_locations, l_leg);

        // Assert
        let theta1 = joint_angles[0][0];
        let theta2 = joint_angles[0][1];
        let x = -l1 * (-theta1).sin() + l2 * (theta2 + theta1).sin();
        let z = -l1 * (-theta1).cos() - l2 * (theta2 + theta1).cos();
        assert_close!(x, x_target, 1e-5);
        assert_close!(z, z_target, 1e-5);
    }

    #[test]
    fn test_inverse_kinematics_5() {
        // Arrange
        let l_leg = 1.0;
        let l1 = l_leg / 2.0;
        let l2 = l_leg / 2.0;
        let x_target = 0.1;
        let z_target = -0.5;
        let foot_locations = vec![vector![x_target, 0., z_target]];

        // Act
        let joint_angles = QuadrupedTrottingController::inverse_kinematics(&foot_locations, l_leg);

        // Assert
        let theta1 = joint_angles[0][0];
        let theta2 = joint_angles[0][1];
        let x = -l1 * (-theta1).sin() + l2 * (theta2 + theta1).sin();
        let z = -l1 * (-theta1).cos() - l2 * (theta2 + theta1).cos();
        assert_close!(x, x_target, 1e-5);
        assert_close!(z, z_target, 1e-5);
    }

    #[test]
    fn quadruped_trot_to_position() {
        // Arrange
        let dt = 1.0 / (60.0 * 50.0);
        let target_x = -0.2;

        let l_leg = 1.0;
        let mut state = build_quadruped();

        let default_z = 0.8;
        let initial_x = -1.2;
        state.set_joint_q(
            1,
            JointPosition::Pose(Pose {
                rotation: UnitQuaternion::identity(),
                translation: vector![initial_x, 0., default_z],
            }),
        );

        let default_foot_stance = vec![
            vector![0., 0., -default_z],
            vector![0., 0., -default_z],
            vector![0., 0., -default_z],
            vector![0., 0., -default_z],
        ];
        let default_joint_angles =
            QuadrupedTrottingController::inverse_kinematics(&default_foot_stance, l_leg);

        for (leg_index, hip_knee_angles) in default_joint_angles.iter().enumerate() {
            let hip_index = leg_index * 2 + 1;
            let knee_index = hip_index + 1;

            let hip_angle = hip_knee_angles[0];
            let knee_angle = hip_knee_angles[1];

            state.set_joint_q(hip_index + 1, JointPosition::Float(hip_angle));
            state.set_joint_q(knee_index + 1, JointPosition::Float(knee_angle));
        }

        let alpha = 1.0;
        let mu = 1.0;
        let ground = HalfSpace::new_with_params(Vector3::z_axis(), 0.0, alpha, mu);
        state.add_halfspace(ground);

        let mut controller = QuadrupedTrottingController::new(dt, target_x, -default_z);

        // Act
        let final_time = 3.0;
        let num_steps = (final_time / dt) as usize;
        let mut final_body_pose = Pose::identity();
        let mut final_body_vel = Vector3::zeros();
        for _ in 0..num_steps {
            let tau = controller.control(&mut state, None);
            let (q, v) = step(&mut state, dt, &tau, &Integrator::SemiImplicitEuler);

            final_body_pose = *q[0].pose();
            final_body_vel = final_body_pose.rotation.inverse() * v[0].spatial().linear;
        }

        // Assert
        assert_close!(final_body_pose.translation.x, target_x, 1e-1);
        assert_close!(final_body_vel.x, 0.0, 3e-1);
    }
}
