use gorilla_physics::joint::ToJointPositionVec;
use gorilla_physics::joint::ToJointVelocityVec;
use gorilla_physics::plot::plot;
use gorilla_physics::{
    control::swingup::swingup_acrobot, energy::double_pendulum_potential_energy2,
    helpers::build_double_pendulum, joint::ToFloatDVec, simulate::step, types::Float,
};
use nalgebra::Isometry3;
use nalgebra::Vector3;
use nalgebra::{dvector, vector, DVector, Matrix3};

/// Acrobot swing up example
pub fn main() {
    let m = 1.0;
    let l = 7.0;

    let moment_x = 0.0;
    let moment_y = m * l * l;
    let moment_z = m * l * l;
    let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
    let cross_part = vector![m * l, 0., 0.];

    let rod1_to_world = Isometry3::identity();
    let rod2_to_rod1 = Isometry3::translation(l, 0., 0.);
    let axis = -Vector3::y_axis();

    let mut state = build_double_pendulum(
        m,
        &moment,
        &cross_part,
        &rod1_to_world,
        &rod2_to_rod1,
        &axis,
    );

    let q_init = vec![0.0, 0.].to_joint_pos_vec();
    let v_init = vec![0., 0.].to_joint_vel_vec();
    state.update(&q_init, &v_init);

    // Act
    let mut qs: DVector<DVector<Float>> = dvector![];
    let mut vs: DVector<DVector<Float>> = dvector![];
    let mut taus: DVector<DVector<Float>> = dvector![];
    let mut data1 = vec![];

    let mut t = 0.0;
    let final_time = 30.0;
    let dt = 1.0 / 1000.0;
    let num_steps = (final_time / dt) as usize;
    while t < final_time {
        let torque = swingup_acrobot(&state, m, l);
        let (q, v) = step(
            &mut state,
            dt,
            &torque,
            &gorilla_physics::integrators::Integrator::SemiImplicitEuler,
        );
        qs.extend([q.to_float_dvec()]);
        vs.extend([v.to_float_dvec()]);
        taus.extend([torque.to_float_dvec()]);

        data1.push(state.kinetic_energy() + double_pendulum_potential_energy2(&state, m, l));

        t += dt;
    }

    plot(&data1, final_time, dt, num_steps, "acrobot swingup energy");
}
