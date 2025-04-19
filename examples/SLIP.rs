use gorilla_physics::{
    contact::HalfSpace,
    control::Controller,
    helpers::build_SLIP,
    interface::controller::NullController,
    joint::{JointPosition, JointVelocity},
    plot::{plot, plot_trajectory},
    simulate::step,
    spatial::pose::Pose,
    spatial::spatial_vector::SpatialVector,
    types::Float,
};
use nalgebra::{vector, UnitQuaternion, UnitVector3, Vector3};

/// Spring Loaded Inverted Pendulum (SLIP)
pub fn main() {
    let m = 0.54;
    let r = 1.0;
    let l_rest = 0.2;
    let v_x_init = 5.0;

    let degree = 45.0;
    let angle: Float = Float::to_radians(degree);
    let f_n = 3.0; // Hz, natural frequency of vertical motion in stance
                   // TODO: do the following to set k_spring.
                   // It is surprisingly difficult to find a set of parameters
                   // to make the SLIP go into periodic motion.
                   // let k_spring = 1.0 * (2.0 * PI * f_n).powi(2) * m;
                   // because w_n = sqrt(k/m); f_n = w_n/2pi;
                   // Ref: A Planar Hopping Robot with One Actuator, Sato & Buehler
    let k_spring = 500.0;

    let mut state = build_SLIP(m, r, l_rest, angle, k_spring);

    let h_ground = -0.3;
    state.add_halfspace(&HalfSpace::new(Vector3::z_axis(), h_ground));

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
    let final_time = 5.00;
    let dt = 1.0 / 600.0;
    let num_steps = (final_time / dt) as usize;

    let mut controller = NullController {};

    let mut data1: Vec<Float> = Vec::with_capacity(num_steps);
    let mut data2: Vec<Float> = Vec::with_capacity(num_steps);
    let mut data3: Vec<Float> = Vec::with_capacity(num_steps);
    let mut v_z_prev = 0.0;

    for s in 0..num_steps {
        let torque = controller.control(&mut state, None);
        let (q, v) = step(
            &mut state,
            dt,
            &torque,
            &gorilla_physics::integrators::Integrator::SemiImplicitEuler,
        );

        let pose_body = q[0].pose();
        let rot = pose_body.rotation;
        let v_linear = rot * v[0].spatial().linear;
        let v_z = v_linear.z;

        if v_z_prev >= 0.0 && v_z < 0.0 {
            let direction = UnitVector3::new_normalize(vector![angle.sin(), 0., -angle.cos()]);

            state.bodies[0].spring_contacts[0].direction = direction;
            state.bodies[0].spring_contacts[0].l_rest = l_rest;
        }

        v_z_prev = v_z;

        let energy = state.kinetic_energy() + state.gravitational_energy();

        data1.push(pose_body.translation.x);
        data2.push(pose_body.translation.z);
        data3.push(energy);
    }

    plot(&data3, final_time, dt, num_steps, "plot");
    plot_trajectory(&data1, &data2, "traj");
}
