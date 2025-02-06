use crate::joint::JointTorque;
use crate::joint::ToJointTorqueVec;
use crate::{energy::double_pendulum_potential_energy2, PI, TWO_PI};
use crate::{mechanism::MechanismState, types::Float, GRAVITY};

/// Swing-up controller for a acrobot
/// Reference: ENERGY BASED CONTROL OF A CLASS OF UNDERACTUATED MECHANICAL
/// SYSTEMS by Mark W. Spong, 1996
pub fn swingup_acrobot(state: &MechanismState, m: &Float, l: &Float) -> Vec<JointTorque> {
    let q1 = state.q[0].float();
    let q2 = state.q[1].float();
    let q1dot = state.v[0].float();
    let q2dot = state.v[1].float();

    let KE = state.kinetic_energy();
    let PE = double_pendulum_potential_energy2(&state, m, l);

    let E_target = m * GRAVITY * (l + 2.0 * l);
    let dE = KE + PE - E_target;

    // Nominal control for swinging up
    let k3 = 2.0;
    let mut u_bar = k3 * (dE * q1dot);
    let cap = 10.; // capping nominal control, important
    if u_bar > cap {
        u_bar = cap;
    } else if u_bar < -cap {
        u_bar = -cap;
    }

    // PD closing of 2nd joint towards zero position
    let mut q2 = q2.rem_euclid(TWO_PI);
    if q2 > PI {
        q2 -= TWO_PI;
    }
    let k1 = 2.0;
    let k2 = 2.0;
    let u_pd = -k1 * q2 - k2 * q2dot;

    // Define parameters
    let q1 = q1;
    let m1 = m;
    let m2 = m;
    let lc1 = l;
    let lc2 = l;
    let l1 = l;
    let I1 = 0.0; // moment of inertia about center-of-mass
    let I2 = 0.0; // moment of inertia about center-of-mass
    let c1 = q1.cos();
    let s2 = q2.sin();
    let c2 = q2.cos();
    let c12 = (q1 + q2).cos();
    let m11 = m1 * lc1 * lc1 + m2 * (l1 * l1 + lc2 * lc2 + 2. * l1 * lc2 * c2) + I1 + I2;
    let m22 = m2 * lc2 * lc2 + I2;
    let m12 = m2 * (lc2 * lc2 + l1 * lc2 * c2) + I2;
    let m21 = m12;
    let h1 = -m2 * l1 * lc2 * s2 * q2dot * q2dot - 2. * m2 * l1 * lc2 * s2 * q2dot * q1dot;
    let h2 = m2 * l1 * lc2 * s2 * q1dot * q1dot;
    let phi1 = (m1 * lc1 + m2 * l1) * GRAVITY * c1 + m2 * lc2 * GRAVITY * c12;
    let phi2 = m2 * lc2 * GRAVITY * c12;

    let m22_bar = m22 - m21 * m12 / m11;
    let h2_bar = h2 - m21 * h1 / m11;
    let phi2_bar = phi2 - m21 * phi1 / m11;

    // Computes the torque that sets second joint acceleration to u_bar+u_pd
    let tau = m22_bar * (u_bar + u_pd) + h2_bar + phi2_bar;
    vec![0., tau].to_joint_torque_vec()
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
) -> Vec<JointTorque> {
    let theta = state.q[1].float();
    let theta_dot = state.v[1].float();
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
    let x = state.q[0].float();
    let x_dot = state.v[0].float();
    let Kp = 1.0;
    let Kd = 1.0;
    let u_pd = -Kp * x - Kd * x_dot;

    // Compute force for setting cart acceleration to u
    let u = u_bar + u_pd;
    let f = (m_c + m_p * sin_theta * sin_theta) * u
        - m_p * GRAVITY * sin_theta * cos_theta
        - m_p * l * sin_theta * theta_dot * theta_dot;

    vec![f, 0.0].to_joint_torque_vec()
}

#[cfg(test)]
mod swingup_tests {
    use crate::joint::{ToJointPositionVec, ToJointVelocityVec};
    use itertools::izip;
    use na::{dvector, vector, DVector, Matrix3, Matrix4};

    use crate::{
        energy::cart_pole_energy,
        helpers::{build_cart_pole, build_double_pendulum},
        joint::ToFloatDVec,
        simulate::simulate,
        transform::Transform3D,
        util::assert_dvec_close,
    };

    use super::*;

    #[test]
    fn acrobot_swingup() {
        // Arrange
        let m = 1.0;
        let l: Float = 7.0;

        let moment_x = 0.0;
        let moment_y = m * l * l;
        let moment_z = m * l * l;
        let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
        let cross_part = vector![m * l, 0., 0.];

        let rod1_to_world = Matrix4::identity();
        let rod2_to_rod1 = Transform3D::move_x(l);
        let axis = vector![0.0, -1.0, 0.0];

        let mut state = build_double_pendulum(
            &m,
            &moment,
            &cross_part,
            &rod1_to_world,
            &rod2_to_rod1,
            &axis,
        );

        let q_init = vec![-PI / 2.0 + 0.1, 0.].to_joint_pos_vec();
        let v_init = vec![0., 0.].to_joint_vel_vec();
        state.update(&q_init, &v_init);

        // Act
        let final_time = 50.0;
        let dt = 1.0 / 60.0;
        let swingup = |state: &MechanismState| swingup_acrobot(state, &m, &l);
        let (qs, vs) = simulate(&mut state, final_time, dt, swingup);

        // Assert
        fn check_swungup(q: &DVector<Float>, v: &DVector<Float>) -> bool {
            let q1 = q[0].rem_euclid(TWO_PI);
            let q2 = q[1].rem_euclid(TWO_PI);
            let v1 = v[0];
            let v2 = v[1];
            // A very relaxed check of being swungup
            (q1 - (PI / 2.0)).abs() < 0.5 && q2.abs() < 0.5 && v1.abs() < 0.3 && v2.abs() < 0.3
        }

        let mut swungup = false;
        for (q, v) in izip!(qs.iter(), vs.iter()) {
            if check_swungup(&q.to_float_dvec(), &v.to_float_dvec()) {
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

        let q_init = vec![0.0, 0.1].to_joint_pos_vec();
        let v_init = vec![0.0, 0.0].to_joint_vel_vec();
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
            if check_swungup(&q.to_float_dvec(), &v.to_float_dvec()) {
                swungup = true;
                break;
            }
        }
        assert_eq!(swungup, true);

        // Check that final cart position is near origin, and energy is close to
        // upright pole on a stationary cart
        let cart_q = qs[qs.len() - 1][0].float();
        let cart_v = vs[vs.len() - 1][0].float();
        let E = cart_pole_energy(&state, &m_pole, &l_pole);
        let E_expected = m_pole * l_pole * GRAVITY;
        assert_dvec_close(&dvector![*cart_q, *cart_v], &dvector![0.0, 0.0], 1e-1);
        assert_dvec_close(&dvector![E], &dvector![E_expected], 1e-1);
    }
}
