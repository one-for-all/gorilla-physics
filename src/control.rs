use crate::{util::console_log, PI};
use na::{dvector, DVector};

use crate::{mechanism::MechanismState, types::Float, GRAVITY};

/// Control algorithm that effectively inverts the gravity for a pendulum system.
/// u = 2mglsin(q) - Bv
/// where q is the angle from the bottom, and Bv is the damping term.
///
/// Reference: https://underactuated.csail.mit.edu/pend.html#section2
pub fn pendulum_gravity_inversion(state: &MechanismState) -> DVector<Float> {
    let mass = state.bodies[0].inertia.mass;
    let length_to_com = state.bodies[0].inertia.center_of_mass().coords.norm();
    let q = state.q[0];

    let gravity_inversion = 2.0 * mass * GRAVITY * length_to_com * q.sin();
    let damping = -10.0 * state.v[0];
    let torque = gravity_inversion + damping;
    dvector![torque]
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
pub fn pendulum_energy_shaping(state: &MechanismState) -> DVector<Float> {
    let mass = state.bodies[0].inertia.mass;
    let length_to_com = state.bodies[0].inertia.center_of_mass().coords.norm();
    let moment = state.bodies[0].inertia.moment;

    let axis = state.treejoints[0].axis;

    let q = state.q[0];
    let v = state.v[0];
    let omega = axis * v;

    let E_desired = mass * GRAVITY * length_to_com;
    let KE = (0.5 * omega.transpose() * moment * omega)[0];
    let PE = mass * GRAVITY * length_to_com * (-q.cos());
    let E_diff = KE + PE - E_desired;

    let torque = -0.1 * v * E_diff;
    dvector![torque]
}

pub fn pendulum_swing_up_and_balance(state: &MechanismState) -> DVector<Float> {
    let q = state.q[0];
    if (q - PI).abs() > 0.15 {
        pendulum_energy_shaping(state) // Swing up
    } else {
        pendulum_gravity_inversion(state) // Balance
    }
}

#[cfg(test)]
mod control_tests {
    use na::{dvector, vector, Matrix3, Matrix4};

    use crate::{simulate::simulate, PI};

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

        let rod_to_world = Matrix4::identity(); // transformation from rod to world frame
        let axis = vector![0.0, 1.0, 0.0]; // axis of joint rotation

        let mut state =
            crate::helpers::build_pendulum(&m, &moment, &cross_part, &rod_to_world, &axis);

        let q_init = dvector![0.1]; // give it some initial displacement
        let v_init = dvector![0.0];
        state.update(&q_init, &v_init);

        // Act
        let final_time = 200.0;
        let dt = 0.02;
        let (qs, vs) = simulate(&mut state, final_time, dt, pendulum_gravity_inversion);

        // Assert
        let q_final = qs.as_slice().last().unwrap()[0];
        let q_error = q_final - PI;
        assert!(
            q_error.abs() < 1e-4,
            "Pendulum should swing to the top. q error: {}",
            q_error
        );

        let v_final = vs.as_slice().last().unwrap()[0];
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

        let rod_to_world = Matrix4::identity(); // transformation from rod to world frame
        let axis = vector![0.0, 1.0, 0.0]; // axis of joint rotation

        let mut state =
            crate::helpers::build_pendulum(&m, &moment, &cross_part, &rod_to_world, &axis);

        let q_init = dvector![0.0];
        let v_init = dvector![0.1]; // give it some initial velocity
        state.update(&q_init, &v_init);

        // Act
        let final_time = 100.0;
        let dt = 0.01;
        let (qs, _vs) = simulate(&mut state, final_time, dt, pendulum_energy_shaping);

        // Assert
        let q_max = qs
            .iter()
            .map(|q| q[0])
            .fold(Float::NEG_INFINITY, Float::max);
        assert!(
            q_max > 3.0,
            "Pendulum should swing to near the top. q_max: {}",
            q_max
        );
    }
}
