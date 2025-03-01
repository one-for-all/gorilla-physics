use gorilla_physics::helpers::build_cart;
use gorilla_physics::plot::plot;
use nalgebra::{vector, Matrix3};

use gorilla_physics::joint::{JointVelocity, ToJointTorqueVec};
use gorilla_physics::{simulate::simulate, types::Float};

/// Run simulation of a cart, which moves along the x-axis
///
///              z
///              |
///              |----> x
///          _________
///         |_________|
pub fn main() {
    let m = 3.0; // mass of cart
    let l = 5.0; // length of cart

    let moment_x = 0.0;
    let moment_y = m * l * l / 12.0;
    let moment_z = m * l * l / 12.0;
    let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
    let cross_part = vector![0.0, 0.0, 0.0];
    let axis = vector![1.0, 0.0, 0.0];

    let mut state = build_cart(&m, &moment, &cross_part, &axis);
    state.set_joint_v(1, JointVelocity::Float(1.0));

    // Simulate
    let final_time = 10.0;
    let dt = 0.02;
    let num_steps = (final_time / dt) as usize;

    let (qs, vs) = simulate(&mut state, final_time, dt, |_state| {
        vec![0.0].to_joint_torque_vec()
    });
    let data = qs.iter().map(|x| *x[0].float()).collect::<Vec<Float>>();

    plot(&data, final_time, dt, num_steps, "plot");
}
