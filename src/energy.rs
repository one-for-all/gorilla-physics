use crate::{mechanism::MechanismState, types::Float, GRAVITY};

/// Compute the gravitational potential energy of a simple double pendulum system
pub fn double_pendulum_potential_energy(state: &MechanismState, m: &Float, l: &Float) -> Float {
    let q1 = state.q[0];
    let q2 = state.q[1];

    let h1 = -l * q1.cos();
    let h2 = -l * q1.cos() - l * (q1 + q2).cos();
    m * GRAVITY * (h1 + h2)
}

/// Compute double pendulum system total energy
pub fn double_pendulum_energy(state: &MechanismState, m: &Float, l: &Float) -> Float {
    state.kinetic_energy() + double_pendulum_potential_energy(state, m, l)
}

/// Compute cart-pole system total energy
pub fn cart_pole_energy(state: &MechanismState, m_p: &Float, l: &Float) -> Float {
    let theta = state.q[1];
    let cos_theta = theta.cos();
    let KE = state.kinetic_energy();
    let PE = -m_p * GRAVITY * l * cos_theta;

    KE + PE
}

#[cfg(test)]
mod energy_tests {
    use std::f32::consts::PI;

    use na::{dvector, vector, Matrix3, Matrix4};

    use crate::{
        energy::double_pendulum_energy,
        helpers::{build_cart_pole, build_double_pendulum},
        simulate::simulate,
        transform::Transform3D,
        types::Float,
        util::assert_close,
    };

    use super::cart_pole_energy;

    #[test]
    fn test_double_pendulum_energy() {
        // Arrange
        let m = 1.0;
        let l: Float = 1.0;

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

        let q_init = dvector![1., 1.];
        let v_init = dvector![1., 1.];
        state.update(&q_init, &v_init);

        let init_energy = double_pendulum_energy(&state, &m, &l);

        // Act
        let final_time = 2.0;
        let dt = 0.001;
        let (_qs, _vs) = simulate(&mut state, final_time, dt, |_state| dvector![0.0, 0.0]);

        // Assert
        let final_energy = double_pendulum_energy(&state, &m, &l);
        assert_close(&dvector![final_energy], &dvector![init_energy], 1e-1);
        // TODO: check for lower tolerance when better integration scheme is implemented
    }

    #[test]
    fn test_cart_pole_energy() {
        let m_cart = 1.0;
        let l_cart = 1.0;
        let moment_x = 0.0;
        let moment_y = m_cart * l_cart * l_cart / 12.0;
        let moment_z = m_cart * l_cart * l_cart / 12.0;
        let moment_cart = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
        let cross_part_cart = vector![0.0, 0.0, 0.0];

        let m_pole = 1.0;
        let l_pole = 1.0;
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

        let q_init = dvector![0.0, PI + 0.1];
        let v_init = dvector![0.0, 0.0];
        state.update(&q_init, &v_init);

        let init_energy = cart_pole_energy(&state, &m_pole, &l_pole);

        // Act
        let final_time = 2.0;
        let dt = 0.001;
        let (_qs, _vs) = simulate(&mut state, final_time, dt, |_state| dvector![0.0, 0.0]);

        // Assert
        let final_energy = cart_pole_energy(&state, &m_pole, &l_pole);
        assert_close(&dvector![final_energy], &dvector![init_energy], 1e-1);
        // TODO: check for lower tolerance when better integration scheme is implemented
    }
}
