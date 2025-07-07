use gorilla_physics::joint::ToJointVelocityVec;
use gorilla_physics::plot::plot2;
use gorilla_physics::{
    control::swingup::swingup_cart_pole,
    energy::cart_pole_energy,
    helpers::build_cart_pole,
    joint::{ToFloatDVec, ToJointPositionVec},
    simulate::step,
    PI,
};
use nalgebra::{dvector, vector, DVector, Matrix3};

use gorilla_physics::types::Float;

/// Run simulation of a cart pole
///
///              z
///              |
///              |----> x
///          _________
///         |____|____|
///              |
///              |
///              +
pub fn main() {
    let m_cart = 1.0;
    let l_cart = 1.0;
    let moment_x = 0.0;
    let moment_y = m_cart * l_cart * l_cart / 12.0;
    let moment_z = m_cart * l_cart * l_cart / 12.0;
    let moment_cart = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
    let cross_part_cart = vector![0.0, 0.0, 0.0];

    let m_pole = 2.0;
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

    let q_init = vec![0.0, PI + 0.1].to_joint_pos_vec();
    let v_init = vec![0.0, 0.0].to_joint_vel_vec();
    state.update(&q_init, &v_init);

    let _F_cart = 1.0; // force to be exerted on the cart

    let mut qs: DVector<DVector<Float>> = dvector![];
    let mut vs: DVector<DVector<Float>> = dvector![];
    let mut taus: DVector<DVector<Float>> = dvector![];
    let mut Es: DVector<Float> = dvector![];

    let mut t = 0.0;
    let final_time = 10.0;
    let dt = 1.0 / 100.0;
    let num_steps = (final_time / dt) as usize;
    while t < final_time {
        let torque = swingup_cart_pole(&state, &m_cart, &m_pole, &l_pole);
        // let torque = dvector![0.0, 0.0];
        let (q, v) = step(
            &mut state,
            dt,
            &torque,
            &gorilla_physics::integrators::Integrator::SemiImplicitEuler,
        );
        qs.extend([q.to_float_dvec()]);
        vs.extend([v.to_float_dvec()]);
        taus.extend([torque.to_float_dvec()]);
        Es.extend([cart_pole_energy(&state, &m_pole, &l_pole)]);
        t += dt;
    }

    let index = 1;
    let qs = qs.iter().map(|x| x[index]).collect::<Vec<Float>>();
    let vs = vs.iter().map(|x| x[index]).collect::<Vec<Float>>();
    let taus = taus.iter().map(|x| x[0]).collect::<Vec<Float>>();

    plot2(&qs, &vs, final_time, dt, num_steps);
}
