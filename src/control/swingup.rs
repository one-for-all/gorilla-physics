use crate::{control::double_pendulum_potential_energy, PI, TWO_PI};
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

#[cfg(test)]
mod swingup_tests {
    use itertools::izip;
    use na::{dvector, vector, Matrix3, Matrix4};

    use crate::{helpers::build_double_pendulum, simulate::simulate, transform::Transform3D};

    use super::*;

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
}
