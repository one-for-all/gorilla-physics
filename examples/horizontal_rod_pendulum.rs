use gorilla_physics::{
    inertia::SpatialInertia, joint::revolute::RevoluteJoint, mechanism::MechanismState,
    rigid_body::RigidBody, simulate::simulate, transform::Transform3D, types::Float,
};
use nalgebra::{dvector, vector, Matrix3, Matrix4};
use plotters::prelude::*;

/// Release a horizontal rod pendulum from rest and simulate its motion.
pub fn main() -> Result<(), Box<dyn std::error::Error>> {
    let m = 5.0; // Mass of rod
    let l: Float = 7.0; // Length of rod

    let moment_x = 0.0;
    let moment_y = 1.0 / 3.0 * m * l * l;
    let moment_z = 1.0 / 3.0 * m * l * l;
    let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
    let cross_part = vector![m * l / 2.0, 0.0, 0.0];

    let rod_frame = "rod";
    let world_frame = "world";
    let rod_to_world = Transform3D::new(rod_frame, world_frame, &Matrix4::identity());
    let axis = vector![0.0, 1.0, 0.0];

    let mut state = MechanismState {
        treejoints: dvector![RevoluteJoint {
            init_mat: rod_to_world.mat.clone(),
            transform: rod_to_world,
            axis
        }],
        treejointids: dvector![1],
        bodies: dvector![RigidBody {
            inertia: SpatialInertia {
                frame: rod_frame.to_string(),
                moment,
                cross_part,
                mass: m
            }
        }],
        q: dvector![0.0],
        v: dvector![0.0],
    };

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
        .caption("Angle vs. Time plot", ("sans-serif", 20))
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

    // println!("vs: {:#?}", vs);

    Ok(())
}
