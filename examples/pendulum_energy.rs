use gorilla_physics::joint::ToJointTorqueVec;
use gorilla_physics::plot::plot;
use gorilla_physics::{
    helpers::build_pendulum, mechanism::MechanismState, simulate::step, types::Float, GRAVITY,
};
use nalgebra::{dvector, vector, DVector, Isometry3, Matrix3, Vector3};

fn potential_energy(state: &MechanismState, l: &Float) -> Float {
    let q = state.q[0].float();
    let m = state.bodies.get(0).unwrap().inertia.mass;

    let h = 0.5 * l * -q.sin();
    m * GRAVITY * h
}

#[allow(non_snake_case)]
/// Plot the kinetic and potential energy of a pendulum system during swing
pub fn main() {
    let m = 5.0; // Mass of rod
    let l: Float = 7.0; // Length of rod

    let moment_x = 0.0;
    let moment_y = 1.0 / 3.0 * m * l * l;
    let moment_z = 1.0 / 3.0 * m * l * l;
    let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
    let cross_part = vector![m * l / 2.0, 0.0, 0.0];

    let rod_to_world = Isometry3::identity();
    let axis = Vector3::y_axis();

    let mut state = build_pendulum(&m, &moment, &cross_part, &rod_to_world, axis);

    let mut KEs: DVector<Float> = dvector![];
    let mut PEs: DVector<Float> = dvector![];
    KEs.extend([state.kinetic_energy()]);
    PEs.extend([potential_energy(&state, &l)]);

    let mut t = 0.0;
    let final_time = 10.0;
    let dt = 0.01;
    let num_steps = (final_time / dt) as usize;
    while t < final_time {
        let _ = step(
            &mut state,
            dt,
            &vec![0.0].to_joint_torque_vec(),
            &gorilla_physics::integrators::Integrator::SemiImplicitEuler,
        );
        KEs.extend([state.kinetic_energy()]);
        PEs.extend([potential_energy(&state, &l)]);
        t += dt;
    }

    let data = (KEs + PEs).as_slice().to_vec();

    plot(&data, final_time, dt, num_steps, "pendulum energy");
}
