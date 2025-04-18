use gorilla_physics::{
    contact::HalfSpace,
    control::{pusher_control::PusherController, Controller},
    helpers::build_pusher,
    plot::plot,
    simulate::step,
};
use nalgebra::Vector3;

/// planar pick-and-place robot arm
/// ------
///      |
///     [ ]
///
pub fn main() {
    let mut state = build_pusher();
    let ground = HalfSpace::new(Vector3::z_axis(), 0.0);
    state.add_halfspace(&ground);

    let mut controller = PusherController {};

    println!("Before: cube position: {}", state.poses()[2].translation);

    let final_time = 10.0;
    let dt = 1.0 / 1000.0;
    let num_steps = (final_time / dt) as usize;
    let mut data1 = vec![];
    for ns in 0..num_steps {
        let torque = controller.control(&mut state);
        let (_q, v) = step(
            &mut state,
            dt,
            &torque,
            &gorilla_physics::integrators::Integrator::SemiImplicitEuler,
        );

        data1.push(_q[2].pose().translation.x);
    }

    println!("After: cube position: {}", state.poses()[2].translation);
    println!("After: cube orientation: {}", state.poses()[2].rotation);

    plot(&data1, final_time, dt, num_steps, "cube x");
}
