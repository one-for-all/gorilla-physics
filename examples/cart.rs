use gorilla_physics::helpers::build_cart;
use nalgebra::{dvector, vector, Matrix3};

use gorilla_physics::{simulate::simulate, types::Float};
use plotters::prelude::*;

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

    let mut state = build_cart(&m, &moment, &cross_part);

    // Simulate
    let final_time = 10.0;
    let dt = 0.02;
    let num_steps = (final_time / dt) as usize;

    let (qs, vs) = simulate(&mut state, final_time, dt, |_state| dvector![0.0]);
    let data = qs.iter().map(|q| q[0]).collect::<Vec<Float>>();

    // Determine y-axis limits based on the minimum and maximum values in the data
    let min_y = data.iter().cloned().fold(Float::INFINITY, Float::min);
    let max_y = data.iter().cloned().fold(Float::NEG_INFINITY, Float::max);

    // Create a plotting area
    let root = BitMapBackend::new("plot.png", (640, 480)).into_drawing_area();
    let _ = root.fill(&WHITE);

    // Configure the chart
    let mut chart = ChartBuilder::on(&root)
        .caption("Position vs. Time plot", ("sans-serif", 20))
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

    // Present the result
    root.present()
        .expect("Unable to present the result to the screen");
}
