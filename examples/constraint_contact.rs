use clarabel::algebra::VectorMath;
use gorilla_physics::{
    helpers::build_cube,
    integrators::Integrator,
    joint::{JointPosition, JointVelocity},
    plot::{plot, plot_trajectory},
    simulate::step,
    spatial::{pose::Pose, spatial_vector::SpatialVector},
    PI,
};
use nalgebra::{vector, UnitQuaternion, Vector3};

/// Drop a box onto floor. Solve contact by constraint-based method
pub fn main() {
    let m = 1.0;
    let l = 1.0;

    let mut state = build_cube(m, l);

    let rot = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), PI / 2.0);
    let q_init = vec![JointPosition::Pose(Pose {
        rotation: rot,
        translation: vector![0.0, 0.0, 1.0],
    })];
    let v_init = vec![JointVelocity::Spatial(SpatialVector {
        angular: vector![0., 0., 0.],
        linear: rot.inverse() * vector![1.0, 0.0, 0.0],
    })];
    state.update(&q_init, &v_init);

    // Simulate
    let final_time = 2.0;
    let dt = 1.0 / 1000.0;
    let num_steps = (final_time / dt) as usize;

    let mut data1 = vec![];
    let mut data2 = vec![];
    let mut data3 = vec![];
    let mut data4 = vec![];
    for _ in 0..num_steps {
        let torque = vec![];
        let (q, v) = step(&mut state, dt, &torque, &Integrator::VelocityStepping);

        let r = q[0].pose().translation;
        let angle = q[0].pose().rotation.angle();
        data1.push(r.x);
        data2.push(r.z);
        let v_box = v[0].spatial();
        let v_box_center = v_box.angular.cross(&r) + v_box.linear;
        data3.push(v_box_center.z);
        data4.push(v_box_center.x);
    }

    println!("max: {}", data2.maximum());

    plot(&data2, final_time, dt, num_steps, "contact z");
    plot(&data3, final_time, dt, num_steps, "contact vz");
    plot(&data4, final_time, dt, num_steps, "contact vx");
    plot_trajectory(&data1, &data2, "constraint contact");
}
