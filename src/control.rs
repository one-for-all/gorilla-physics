use crate::PI;
use na::{dvector, DMatrix, DVector, Matrix1x4};

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

/// PID controller for double pendulum around upright position
/// actually with only P & D terms at the moment
pub fn pid(state: &MechanismState) -> DVector<Float> {
    let q1 = state.q[0];
    let q2 = state.q[1];
    let v1 = state.v[0];
    let v2 = state.v[1];

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

/// Linear Quadratic Regulator for acrobot
/// Reference: https://underactuated.csail.mit.edu/lqr.html
pub fn lqr(state: &MechanismState) -> DVector<Float> {
    // Hard-code with value from lqr.py
    let K = Matrix1x4::<Float>::new(
        -14067.26123453,
        -4689.08739542,
        -15265.74479887,
        -5803.13757768,
    );

    // Stack the position and velocity vectors into a single state vector
    let mut stacked_data = Vec::new();
    stacked_data.extend_from_slice(state.q.as_slice());
    stacked_data.extend_from_slice(state.v.as_slice());
    let x = DMatrix::from_column_slice(4, 1, &stacked_data);

    // Controller takes the form:
    // ubar = -K * xbar
    // where ubar = u - u0, and xbar = x - x0
    // and u0 and x0 are the stabilization operating point
    let mut xbar = x.clone();
    let q1_target = -PI;
    xbar[(0, 0)] = x[(0, 0)] - q1_target;

    let u = -K * xbar;
    dvector![0., u[0]]
}

#[cfg(test)]
mod control_tests {
    use na::{dvector, vector, Matrix3, Matrix4};

    use crate::{
        helpers::build_double_pendulum, simulate::simulate, transform::Transform3D,
        util::assert_close, PI,
    };

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

    #[test]
    fn test_double_pendulum_lqr() {
        // Arrange
        let m = 5.0;
        let l: Float = 7.0;

        let moment_x = m * l * l;
        let moment_y = m * l * l;
        let moment_z = 0.;
        let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
        let cross_part = vector![0., 0., -m * l];

        let rod1_to_world = Matrix4::identity();
        let rod2_to_rod1 = Transform3D::move_z(-l);
        let axis = vector![0.0, 1.0, 0.0]; // axis of joint rotation

        let mut state = build_double_pendulum(
            &m,
            &moment,
            &cross_part,
            &rod1_to_world,
            &rod2_to_rod1,
            &axis,
        );

        let q1_upright = -PI;
        let q_init = dvector![q1_upright - 0.015, 0.];
        let v_init = dvector![0., 0.];
        state.update(&q_init, &v_init); // Set to near upright position

        // Act
        let final_time = 100.0;
        let dt = 0.01;
        let (qs, vs) = simulate(&mut state, final_time, dt, lqr);

        // Assert
        let q_final = &qs[qs.len() - 1];
        let v_final = &vs[vs.len() - 1];
        assert_close(q_final, &dvector![q1_upright, 0.0], 1e-3);
        assert_close(v_final, &dvector![0.0, 0.0], 1e-3);
    }
}
