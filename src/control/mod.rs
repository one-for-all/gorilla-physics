use crate::joint::{JointTorque, ToJointTorqueVec};
use crate::PI;
use na::{dvector, DVector};

use crate::{mechanism::MechanismState, types::Float, GRAVITY};

pub mod SLIP_control;
pub mod energy_control;
pub mod gripper_control;
pub mod hopper_control;
pub mod lqr;
pub mod navbot_conrol;
pub mod pusher_control;
pub mod quadruped_control;
pub mod so101_control;
pub mod swingup;

pub struct ControlInput {
    floats: Vec<Float>,
}

impl ControlInput {
    pub fn new(floats: Vec<Float>) -> Self {
        ControlInput { floats }
    }
}

pub trait Controller {
    fn control(
        &mut self,
        state: &mut MechanismState,
        input: Option<&ControlInput>,
    ) -> Vec<JointTorque>;
}

/// Control algorithm that effectively inverts the gravity for a pendulum system.
/// u = 2mglsin(q) - Bv
/// where q is the angle from the bottom, and Bv is the damping term.
///
/// Reference: https://underactuated.csail.mit.edu/pend.html#section2
pub fn pendulum_gravity_inversion(state: &MechanismState) -> Vec<JointTorque> {
    let mass = state.bodies[0].inertia.mass;
    let length_to_com = state.bodies[0].inertia.center_of_mass().coords.norm();
    let q = state.q[0].float();

    let gravity_inversion = 2.0 * mass * GRAVITY * length_to_com * q.sin();
    let damping = -10.0 * state.v[0].float();
    let torque = gravity_inversion + damping;
    vec![torque].to_joint_torque_vec()
}

/// Control algorithm that pumps energy into the system to achieve the desired
/// energy which places the pendulum at the top.
/// u = -k * qdot * (E - E_desired), k > 0
/// It shapes the energy error dynamics into:
/// E_error_dot = -k * qdot * qdot * E_error
/// which implies an exponential convergence.
///
/// Note: it might not work as amazingly here in sim, due to numerical error.
///
/// Reference: https://underactuated.csail.mit.edu/pend.html#section3
pub fn pendulum_energy_shaping(state: &MechanismState) -> Vec<JointTorque> {
    let mass = state.bodies[0].inertia.mass;
    let length_to_com = state.bodies[0].inertia.center_of_mass().coords.norm();
    let moment = state.bodies[0].inertia.moment;

    let axis = state.treejoints[0].axis();

    let q = state.q[0].float();
    let v = state.v[0].float();
    let omega = axis * *v;

    let E_desired = mass * GRAVITY * length_to_com;
    let KE = (0.5 * omega.transpose() * moment * omega)[0];
    let PE = mass * GRAVITY * length_to_com * (-q.cos());
    let E_diff = KE + PE - E_desired;

    let torque = -0.1 * v * E_diff;
    vec![torque].to_joint_torque_vec()
}

pub fn pendulum_swing_up_and_balance(state: &MechanismState) -> Vec<JointTorque> {
    let q = state.q[0].float();
    if (q - PI).abs() > 0.15 {
        pendulum_energy_shaping(state) // Swing up
    } else {
        pendulum_gravity_inversion(state) // Balance
    }
}

/// PID controller for double pendulum around upright position
/// actually with only P & D terms at the moment
pub fn pid(state: &MechanismState) -> DVector<Float> {
    let q1 = state.q[0].float();
    let q2 = state.q[1].float();
    let v1 = state.v[0].float();
    let v2 = state.v[1].float();

    let kp1 = 5000.0;
    let kd1 = 100.0;
    let kp2 = 5000.0;
    let kd2 = 100.0;

    let q1_desired = -PI;
    let q2_desired = 0.0;
    let q1_error = q1_desired - q1;
    let q2_error = q2_desired - q2;
    let v1_error = 0.0 - v1;
    let v2_error = 0.0 - v2;

    let torque1 = kp1 * q1_error + kd1 * v1_error;
    let torque2 = kp2 * q2_error + kd2 * v2_error;
    dvector![torque1, torque2]
}

#[cfg(test)]
mod control_tests {
    use crate::integrators::Integrator;
    use crate::joint::ToJointPositionVec;
    use crate::joint::ToJointVelocityVec;
    use crate::{simulate::simulate, PI};
    
    use na::Isometry3;
    use na::Vector3;
    use na::{vector, Matrix3};

    use super::*;

    /// Pendulum created at bottom position, and then slightly displaced.
    #[test]
    fn test_pendulum_gravity_inversion() {
        // Arrange
        let m = 5.0;
        let l = 7.0;

        let moment_x = 1.0 / 3.0 * m * l * l;
        let moment_y = 1.0 / 3.0 * m * l * l;
        let moment_z = 0.0;
        let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
        let cross_part = vector![0.0, 0.0, -m * l / 2.0];

        let rod_to_world = Isometry3::identity(); // transformation from rod to world frame
        let axis = Vector3::y_axis(); // axis of joint rotation

        let mut state =
            crate::helpers::build_pendulum(&m, &moment, &cross_part, &rod_to_world, axis);

        let q_init = vec![0.1].to_joint_pos_vec(); // give it some initial displacement
        let v_init = vec![0.0].to_joint_vel_vec();
        state.update(&q_init, &v_init);

        // Act
        let final_time = 200.0;
        let dt = 0.01;
        let (qs, vs) = simulate(
            &mut state,
            final_time,
            dt,
            pendulum_gravity_inversion,
            &Integrator::RungeKutta4,
        );

        // Assert
        let q_final = qs.as_slice().last().unwrap()[0].float();
        let q_error = q_final - PI;
        assert!(
            q_error.abs() < 1e-3,
            "Pendulum should swing to the top. q error: {}",
            q_error
        );

        let v_final = vs.as_slice().last().unwrap()[0].float();
        assert!(
            v_final.abs() < 1e-4,
            "Pendulum should stop at the top, v: {}",
            v_final
        );
    }

    #[test]
    fn test_pendulum_energy_shaping() {
        // Arrange
        let m = 5.0;
        let l = 7.0;

        let moment_x = 1.0 / 3.0 * m * l * l;
        let moment_y = 1.0 / 3.0 * m * l * l;
        let moment_z = 0.0;
        let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
        let cross_part = vector![0.0, 0.0, -m * l / 2.0];

        let rod_to_world = Isometry3::identity(); // transformation from rod to world frame
        let axis = Vector3::y_axis(); // axis of joint rotation

        let mut state =
            crate::helpers::build_pendulum(&m, &moment, &cross_part, &rod_to_world, axis);

        let q_init = vec![0.0].to_joint_pos_vec();
        let v_init = vec![0.1].to_joint_vel_vec(); // give it some initial velocity
        state.update(&q_init, &v_init);

        // Act
        let final_time = 50.0;
        let dt = 0.01;
        let (qs, _vs) = simulate(
            &mut state,
            final_time,
            dt,
            pendulum_energy_shaping,
            &Integrator::SemiImplicitEuler,
        );

        // Assert
        let q_max = qs
            .iter()
            .map(|q| q[0].float().clone())
            .fold(Float::NEG_INFINITY, Float::max);
        assert!(
            q_max > 3.0,
            "Pendulum should swing to near the top. q_max: {}",
            q_max
        );
    }
}
