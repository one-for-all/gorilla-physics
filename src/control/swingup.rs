use crate::{energy::double_pendulum_potential_energy, PI, TWO_PI};
use na::{dvector, DVector};

use crate::{mechanism::MechanismState, types::Float, GRAVITY};

/// Swing-up controller for a acrobot
/// Reference: ENERGY BASED CONTROL OF A CLASS OF UNDERACTUATED MECHANICAL
/// SYSTEMS by Mark W. Spong, 1996
pub fn swingup_acrobot(state: &MechanismState, m: &Float, l: &Float) -> DVector<Float> {
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

    dvector![0., (u_pd + ubar)]
}

/// Swing-up controller for cart-pole system
/// Ref:
///     1. https://underactuated.csail.mit.edu/acrobot.html#section6
///     2. Nonlinear Control of a Swinging Pendulum by CHUNG CHOO CHUNG and JOHN
///     HAUSER, 1995
pub fn swingup_cart_pole(
    state: &MechanismState,
    m_c: &Float,
    m_p: &Float,
    l: &Float,
) -> DVector<Float> {
    let theta = state.q[1];
    let theta_dot = state.v[1];
    let cos_theta = theta.cos();
    let sin_theta = theta.sin();
    let KE = 0.5 * m_p * l * l * theta_dot * theta_dot; // KE of the pole
    let PE = -m_p * GRAVITY * l * cos_theta;

    let E_target = m_p * GRAVITY * l;
    let dE = KE + PE - E_target;

    // Nominal control for swinging up
    let K = 2.0;
    let u_bar = K * theta_dot * cos_theta * dE / (m_p * l);

    // PD closing of cart position error
    let x = state.q[0];
    let x_dot = state.v[0];
    let Kp = 1.0;
    let Kd = 1.0;
    let u_pd = -Kp * x - Kd * x_dot;

    // Compute force for setting cart acceleration to u
    let u = u_bar + u_pd;
    let f = (m_c + m_p * sin_theta * sin_theta) * u
        - m_p * GRAVITY * sin_theta * cos_theta
        - m_p * l * sin_theta * theta_dot * theta_dot;

    dvector![f, 0.0]
}

#[cfg(test)]
mod swingup_tests {
    use itertools::izip;
    use na::{dvector, vector, Matrix3, Matrix4};

    use crate::{
        energy::cart_pole_energy,
        helpers::{build_cart_pole, build_double_pendulum},
        simulate::simulate,
        transform::Transform3D,
        util::assert_close,
    };

    use super::*;

    #[ignore]
    #[test]
    fn acrobot_swingup() {
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
        let axis = vector![0.0, 1.0, 0.0];

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
        let swingup = |state: &MechanismState| swingup_acrobot(state, &m, &l);
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

    #[test]
    fn cart_pole_swingup() {
        let m_cart = 3.0;
        let l_cart = 1.0;
        let moment_x = 0.0;
        let moment_y = m_cart * l_cart * l_cart / 12.0;
        let moment_z = m_cart * l_cart * l_cart / 12.0;
        let moment_cart = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
        let cross_part_cart = vector![0.0, 0.0, 0.0];

        let m_pole = 5.0;
        let l_pole = 7.0;
        let moment_x = m_pole * l_pole * l_pole;
        let moment_y = m_pole * l_pole * l_pole;
        let moment_z = 0.0;
        let moment_pole = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
        let cross_part_pole = vector![0.0, 0.0, -l_pole * m_pole];
        let axis_pole = vector![0.0, -1.0, 0.0];

        let mut state = build_cart_pole(
            &m_cart,
            &m_pole,
            &moment_cart,
            &moment_pole,
            &cross_part_cart,
            &cross_part_pole,
            &axis_pole,
        );

        let q_init = dvector![0.0, 0.1];
        let v_init = dvector![0.0, 0.0];
        state.update(&q_init, &v_init);

        // Act
        let final_time = 50.0;
        let dt = 0.01;
        let swingup = |state: &MechanismState| swingup_cart_pole(state, &m_cart, &m_pole, &l_pole);
        let (qs, vs) = simulate(&mut state, final_time, dt, swingup);

        // Assert
        // Check that the pole had been swung up near origin of cart
        fn check_swungup(q: &DVector<Float>, v: &DVector<Float>) -> bool {
            let q1 = q[0];
            let q2 = q[1].rem_euclid(TWO_PI);
            let v1 = v[0];
            let v2 = v[1];
            q1.abs() < 1e-3 && (q2 - PI).abs() < 1e-1 && v1.abs() < 1e-2 && v2.abs() < 1e-1
        }
        let mut swungup = false;
        for (q, v) in izip!(qs.iter(), vs.iter()) {
            if check_swungup(&q, &v) {
                swungup = true;
                break;
            }
        }
        assert_eq!(swungup, true);

        // Check that final cart position is near origin, and energy is close to
        // upright pole on a stationary cart
        let cart_q = qs[qs.len() - 1][0];
        let cart_v = vs[vs.len() - 1][0];
        let E = cart_pole_energy(&state, &m_pole, &l_pole);
        let E_expected = m_pole * l_pole * GRAVITY;
        assert_close(&dvector![cart_q, cart_v], &dvector![0.0, 0.0], 1e-1);
        assert_close(&dvector![E], &dvector![E_expected], 1e-1);
    }
}
