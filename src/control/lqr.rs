use crate::PI;
use na::{dvector, DMatrix, DVector, Matrix1x4};

use crate::{mechanism::MechanismState, types::Float};

/// Linear Quadratic Regulator for acrobot
/// Reference: https://underactuated.csail.mit.edu/lqr.html
pub fn lqr_acrobot(state: &MechanismState) -> DVector<Float> {
    // Hard-code with value from lqr_acrobot.py
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

/// Linear Quadratic Regulator for cart-pole
/// Reference: https://underactuated.csail.mit.edu/lqr.html
pub fn lqr_cart_pole(state: &MechanismState) -> DVector<Float> {
    // Hard-code with value from lqr_cart_pole.py
    // Invert the sign of 2nd and 4th values, because the axis in the book formulation is
    // opposite to the system defined here.
    let K = Matrix1x4::<Float>::new(-1., -210.06025784, -5.27501096, -129.54543534);

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
    let angle_target = PI;
    xbar[(1, 0)] = x[(1, 0)] - angle_target;

    let u = -K * xbar;
    dvector![u[0], 0.]
}

#[cfg(test)]
mod lqr_tests {
    use na::{dvector, vector, Matrix3, Matrix4};

    use crate::{
        helpers::{build_cart_pole, build_double_pendulum},
        simulate::simulate,
        transform::Transform3D,
        util::assert_close,
        PI,
    };

    use super::*;

    #[test]
    fn acrobot_lqr() {
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
        let (qs, vs) = simulate(&mut state, final_time, dt, lqr_acrobot);

        // Assert
        let q_final = &qs[qs.len() - 1];
        let v_final = &vs[vs.len() - 1];
        assert_close(q_final, &dvector![q1_upright, 0.0], 1e-3);
        assert_close(v_final, &dvector![0.0, 0.0], 1e-3);
    }

    #[test]
    fn cart_pole_lqr() {
        // Arrange
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

        let mut state = build_cart_pole(
            &m_cart,
            &m_pole,
            &moment_cart,
            &moment_pole,
            &cross_part_cart,
            &cross_part_pole,
        );

        let angle_upright = PI;
        let q_init = dvector![-1., angle_upright + 0.01]; // Set to near upright position
        let v_init = dvector![1., 0.01];
        state.update(&q_init, &v_init);

        // Act
        let final_time = 50.0;
        let dt = 0.01;
        let (qs, vs) = simulate(&mut state, final_time, dt, lqr_cart_pole);

        // Assert
        let q_final = &qs[qs.len() - 1];
        let v_final = &vs[vs.len() - 1];
        assert_close(q_final, &dvector![0.0, angle_upright], 2e-3);
        assert_close(v_final, &dvector![0.0, 0.0], 1e-3);
    }
}
