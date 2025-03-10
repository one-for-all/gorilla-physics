use gorilla_physics::joint::ToJointPositionVec;
use gorilla_physics::joint::ToJointVelocityVec;
use gorilla_physics::plot::plot;
use gorilla_physics::transform::Matrix4Ext;
use gorilla_physics::{
    control::swingup::swingup_acrobot, energy::double_pendulum_potential_energy2,
    helpers::build_double_pendulum, joint::ToFloatDVec, simulate::step, transform::Transform3D,
    types::Float,
};
use nalgebra::{dvector, vector, DVector, Matrix3, Matrix4};

pub fn main() {
    let m = 1.0;
    let l = 7.0;

    let moment_x = 0.0;
    let moment_y = m * l * l;
    let moment_z = m * l * l;
    let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
    let cross_part = vector![m * l, 0., 0.];

    let rod1_to_world = Matrix4::identity();
    let rod2_to_rod1 = Matrix4::<Float>::move_x(l);
    let axis = vector![0.0, -1.0, 0.0];

    let mut state = build_double_pendulum(
        &m,
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
    let mut Es: Vec<Float> = vec![];

    let mut t = 0.0;
    let final_time = 30.0;
    let dt = 1.0 / 1000.0;
    let num_steps = (final_time / dt) as usize;
    while t < final_time {
        let torque = swingup_acrobot(&state, &m, &l);
        let (q, v) = step(
            &mut state,
            dt,
            &torque,
            &gorilla_physics::integrators::Integrator::SemiImplicitEuler,
        );
        qs.extend([q.to_float_dvec()]);
        vs.extend([v.to_float_dvec()]);
        taus.extend([torque.to_float_dvec()]);

        Es.push(state.kinetic_energy() + double_pendulum_potential_energy2(&state, &m, &l));
        t += dt;
    }

    let data = Es;

    plot(&data, final_time, dt, num_steps, "acrobot swingup");
}
