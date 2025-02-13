use gorilla_physics::{
    contact::HalfSpace,
    helpers::build_cube,
    joint::{JointPosition, JointVelocity},
    pose::Pose,
    simulate::step,
    spatial_vector::SpatialVector,
    types::Float,
};
use nalgebra::{vector, UnitQuaternion};
use plotters::prelude::*;

pub fn main() {
    let m = 3.0;
    let l = 1.0;
    let mut state = build_cube(m, l);

    let h_ground = -l / 2.0;
    state.add_halfspace(&HalfSpace::new(vector![0., 0., 1.], h_ground));

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
    let final_time = 5.0;
    let dt = 1.0 / 5000.0;
    let num_steps = (final_time / dt) as usize;

    let mut data: Vec<Float> = Vec::with_capacity(num_steps);
    for _ in 0..num_steps {
        let torque = vec![];
        let (q, v) = step(&mut state, dt, &torque);

        data.push(v[0].spatial().linear.x);
    }

    // Determine y-axis limits based on the minimum and maximum values in the data
    let min_y = data.iter().cloned().fold(Float::INFINITY, Float::min);
    let max_y = data.iter().cloned().fold(Float::NEG_INFINITY, Float::max);

    // Create a plotting area
    let root = BitMapBackend::new("plot.png", (640, 480)).into_drawing_area();
    let _ = root.fill(&WHITE);

    // Configure the chart
    let mut chart = ChartBuilder::on(&root)
        .caption("x vs. Time plot", ("sans-serif", 20))
        .x_label_area_size(30)
        .y_label_area_size(40)
        .build_cartesian_2d(0.0..final_time, min_y..max_y)
        .unwrap();

    // Customize the chart
    let _ = chart.configure_mesh().draw();

    // Plot the data
    let _ = chart.draw_series(LineSeries::new(
        (0..num_steps).map(|i| (i as Float * dt, data[i])),
        &BLUE,
    ));
}
