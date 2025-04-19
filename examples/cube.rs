use gorilla_physics::{
    contact::HalfSpace,
    helpers::build_cube,
    joint::{JointPosition, JointVelocity},
    plot::{plot, plot_trajectory},
    simulate::step,
    spatial::pose::Pose,
    spatial::spatial_vector::SpatialVector,
    types::Float,
};
use nalgebra::{vector, UnitQuaternion, Vector3};

/// Cube hitting ground with x velocity
pub fn main() {
    let m = 3.0;
    let l = 1.0;
    let v_x_init = 1.0;
    let mut state = build_cube(m, l);

    let h_ground = -l / 2.0 - 0.5;

    let alpha = 1.0; // velocity lost
    let mu = 1.5; // coefficient of friction
    let ground = HalfSpace::new_with_params(Vector3::z_axis(), h_ground, alpha, mu);
    state.add_halfspace(&ground);

    let q_init = vec![JointPosition::Pose(Pose {
        rotation: UnitQuaternion::identity(),
        translation: vector![0.0, 0.0, 0.0],
    })];
    let v_init = vec![JointVelocity::Spatial(SpatialVector {
        angular: vector![0.0, 0.0, 0.0],
        linear: vector![v_x_init, 0.0, 0.0],
    })];
    state.update(&q_init, &v_init);

    // Simulate
    let final_time = 5.0;
    let dt = 1.0 / 5000.0;
    let num_steps = (final_time / dt) as usize;

    let mut data1: Vec<Float> = Vec::with_capacity(num_steps);
    let mut data2: Vec<Float> = Vec::with_capacity(num_steps);
    for _ in 0..num_steps {
        let torque = vec![];
        let (q, _v) = step(
            &mut state,
            dt,
            &torque,
            &gorilla_physics::integrators::Integrator::SemiImplicitEuler,
        );

        data1.push(q[0].pose().translation.x);
        data2.push(q[0].pose().translation.z);
    }

    plot(&data1, final_time, dt, num_steps, "plot");
    plot_trajectory(&data1, &data2, "traj");
}
