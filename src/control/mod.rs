use crate::{PI, TWO_PI};
use na::{dvector, DVector};

use crate::{mechanism::MechanismState, types::Float, GRAVITY};

pub mod lqr;

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

    let axis = state.treejoints[0].axis();

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

/// Compute the gravitational potential energy of a simple double pendulum system
fn double_pendulum_potential_energy(state: &MechanismState, m: &Float, l: &Float) -> Float {
    let q1 = state.q[0];
    let q2 = state.q[1];

    let h1 = -l * q1.cos();
    let h2 = -l * q1.cos() - l * (q1 + q2).cos();
    m * GRAVITY * (h1 + h2)
}

/// Swing-up controller for a double pendulum
/// Reference: ENERGY BASED CONTROL OF A CLASS OF UNDERACTUATED MECHANICAL
/// SYSTEMS by Mark W. Spong, 1996
pub fn double_pendulum_swingup(state: &MechanismState, m: &Float, l: &Float) -> DVector<Float> {
    let KE = state.kinetic_energy();
    let PE = double_pendulum_potential_energy(&state, m, l);

    let E_target = m * GRAVITY * (l + 2.0 * l);
    let dE = KE + PE - E_target;

    // Nominal control for swinging up
    let v1 = state.v[0];
    let k3 = 2.0;
    let mut ubar = k3 * (dE * v1);
    let cap = 100.; // capping nominal control
    if ubar > cap {
        ubar = cap;
    } else if ubar < -cap {
        ubar = -cap;
    }

    // PD closing of 2nd joint towards zero position
    let mut q2 = state.q[1].rem_euclid(TWO_PI);
    if q2 > PI {
        q2 -= TWO_PI;
    }
    let v2 = state.v[1];
    let k1 = 10.0;
    let k2 = 10.0;
    let u_pd = -k1 * q2 - k2 * v2;

    dvector![0., u_pd + ubar]
}

#[cfg(test)]
mod control_tests {
    use itertools::izip;
    use na::{dvector, vector, Matrix3, Matrix4};

    use crate::{helpers::build_double_pendulum, simulate::simulate, transform::Transform3D, PI};

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
    fn test_double_pendulum_swingup() {
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

        let q_init = dvector![0.01, 0.];
        let v_init = dvector![0., 0.];
        state.update(&q_init, &v_init);

        // Act
        let final_time = 50.0;
        let dt = 0.01;
        let swingup = |state: &MechanismState| double_pendulum_swingup(state, &m, &l);
        let (qs, vs) = simulate(&mut state, final_time, dt, swingup);

        // Assert
        fn check_swungup(q: &DVector<Float>, v: &DVector<Float>) -> bool {
            let q1 = q[0].rem_euclid(TWO_PI);
            let q2 = q[1].rem_euclid(TWO_PI);
            let v1 = v[0];
            let v2 = v[1];
            // A very relaxed check of being swungup
            (q1 - PI).abs() < 1. && q2.abs() < 1. && v1.abs() < 0.8 && v2.abs() < 0.8
        }

        let mut swungup = false;
        for (q, v) in izip!(qs.iter(), vs.iter()) {
            if check_swungup(&q, &v) {
                swungup = true;
                break;
            }
        }
        assert_eq!(swungup, true);
    }
}
