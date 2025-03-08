use gorilla_physics::{
    contact::HalfSpace,
    helpers::build_rimless_wheel,
    joint::{JointPosition, JointVelocity},
    plot::plot,
    pose::Pose,
    simulate::step,
    spatial_vector::SpatialVector,
    types::Float,
    GRAVITY, PI,
};
use nalgebra::{vector, UnitQuaternion, UnitVector3, Vector3};

pub fn main() {
    let m_body = 10.0;
    let r_body = 5.0;

    let l = 10.0;
    let n_foot = 8;

    let alpha = 2.0 * PI / n_foot as Float / 2.0;
    let mut state = build_rimless_wheel(m_body, r_body, l, n_foot);

    let h_ground = -20.0;
    let angle: Float = Float::to_radians(10.0);
    let normal = UnitVector3::new_normalize(vector![angle.sin(), 0.0, angle.cos()]);
    state.add_halfspace(&HalfSpace::new(normal, h_ground));

    let q_init = vec![JointPosition::Pose(Pose {
        rotation: UnitQuaternion::identity(),
        translation: vector![0.0, 0.0, 0.0],
    })];
    let v_init = vec![JointVelocity::Spatial(SpatialVector {
        angular: vector![0.0, 0.0, 0.0],
        linear: vector![1.0, 0.0, 0.0],
    })];
    state.update(&q_init, &v_init);

    // Simulate
    let final_time = 20.0;
    let dt = 1.0 / 600.0;
    let num_steps = (final_time / dt) as usize;

    let mut data: Vec<Float> = Vec::with_capacity(num_steps);
    for s in 0..num_steps {
        let torque = vec![];
        let (q, v) = step(&mut state, dt, &torque);

        data.push(v[0].spatial().angular.dot(&Vector3::y_axis()));
    }

    // theoretical angular velocity right after each collision, for an ideal
    // point-mass rimless wheel
    // Ref: Underactuated Robotics. https://underactuated.mit.edu/simple_legs.html
    let omega =
        (1.0 / (2.0 * alpha).tan()) * (4.0 * GRAVITY / l * alpha.sin() * angle.sin()).sqrt();
    println!("omega: {}", omega);

    plot(&data, final_time, dt, num_steps, "rimless wheel");
}
