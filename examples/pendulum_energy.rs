use gorilla_physics::{
    helpers::build_pendulum, mechanism::MechanismState, simulate::step, types::Float, GRAVITY,
};
use nalgebra::{dvector, vector, DVector, Matrix3, Matrix4};
use plotters::prelude::*;

fn potential_energy(state: &MechanismState, l: &Float) -> Float {
    let q = state.q[0];
    let m = state.bodies.get(0).unwrap().inertia.mass;

    let h = 0.5 * l * -q.sin();
    m * GRAVITY * h
}
/// Plot the kinetic and potential energy of a pendulum system during swing
pub fn main() {
    let m = 5.0; // Mass of rod
    let l: Float = 7.0; // Length of rod

    let moment_x = 0.0;
    let moment_y = 1.0 / 3.0 * m * l * l;
    let moment_z = 1.0 / 3.0 * m * l * l;
    let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
    let cross_part = vector![m * l / 2.0, 0.0, 0.0];

    let rod_to_world = Matrix4::identity();
    let axis = vector![0.0, 1.0, 0.0];

    let mut state = build_pendulum(&m, &moment, &cross_part, &rod_to_world, &axis);

    let mut KEs: DVector<Float> = dvector![];
    let mut PEs: DVector<Float> = dvector![];
    KEs.extend([state.kinetic_energy()]);
    PEs.extend([potential_energy(&state, &l)]);

    let mut t = 0.0;
    let final_time = 10.0;
    let dt = 0.01;
    let num_steps = (final_time / dt) as usize;
    while t < final_time {
        let _ = step(&mut state, dt, &dvector![0.0]);
        KEs.extend([state.kinetic_energy()]);
        PEs.extend([potential_energy(&state, &l)]);
        t += dt;
    }

    let data = KEs + PEs;

    // Determine y-axis limits based on the minimum and maximum values in the data
    let min_y = data.iter().cloned().fold(Float::INFINITY, Float::min);
    let max_y = data.iter().cloned().fold(Float::NEG_INFINITY, Float::max);

    // Create a plotting area
    let root = BitMapBackend::new("plot.png", (640, 480)).into_drawing_area();
    let _ = root.fill(&WHITE);

    // Configure the chart
    let mut chart = ChartBuilder::on(&root)
        .caption("Kinetic Energy vs. Time plot", ("sans-serif", 20))
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
