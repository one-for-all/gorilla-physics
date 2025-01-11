use gorilla_physics::helpers::build_cart_pole;
use nalgebra::{dvector, vector, Matrix3};

use gorilla_physics::{simulate::simulate, types::Float};
use plotters::prelude::*;

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
    let m_cart = 3.0;
    let l_cart = 1.0;
    let moment_x = 0.0;
    let moment_y = m_cart * l_cart * l_cart / 12.0;
    let moment_z = m_cart * l_cart * l_cart / 12.0;
    let moment_cart = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
    let cross_part_cart = vector![0.0, 0.0, 0.0];

    let m_pole = 5.0;
    let l_pole = 7.0;
    let moment_x = m_pole * l_pole * l_pole;
    let moment_y = m_pole * l_pole * l_pole;
    let moment_z = 0.0;
    let moment_pole = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
    let cross_part_pole = vector![0.0, 0.0, -l_pole * m_pole];

    let mut state = build_cart_pole(
        &m_cart,
        &m_pole,
        &moment_cart,
        &moment_pole,
        &cross_part_cart,
        &cross_part_pole,
    );

    let F_cart = 1.0; // force to be exerted on the cart
    let final_time = 20.0;
    let dt = 0.02;
    let num_steps = (final_time / dt) as usize;
    let (qs, vs) = simulate(&mut state, final_time, dt, |_state| dvector![F_cart, 0.0]);

    let data = vs.iter().map(|q| q[0]).collect::<Vec<Float>>();

    // Determine y-axis limits based on the minimum and maximum values in the data
    let min_y = data.iter().cloned().fold(Float::INFINITY, Float::min);
    let max_y = data.iter().cloned().fold(Float::NEG_INFINITY, Float::max);

    // Create a plotting area
    let root = BitMapBackend::new("cart_pole_vel.png", (640, 480)).into_drawing_area();
    let _ = root.fill(&WHITE);

    // Configure the chart
    let mut chart = ChartBuilder::on(&root)
        .caption("Cart velocity vs. Time plot", ("sans-serif", 20))
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
