use crate::types::Float;
use plotters::prelude::*;

pub fn plot(data: &Vec<Float>, final_time: Float, dt: Float, num_steps: usize, fname: &str) {
    // Determine y-axis limits based on the minimum and maximum values in the data
    let min_y = data.iter().cloned().fold(Float::INFINITY, Float::min);
    let max_y = data.iter().cloned().fold(Float::NEG_INFINITY, Float::max);

    // Create a plotting area
    let file_name = format!("{}.png", fname);
    let root = BitMapBackend::new(&file_name, (640, 480)).into_drawing_area();
    let _ = root.fill(&WHITE);

    // Configure the chart
    let mut chart = ChartBuilder::on(&root)
        .caption("x vs. Time", ("sans-serif", 20))
        .margin(10)
        .x_label_area_size(30)
        .y_label_area_size(50)
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

pub fn plot2(
    data1: &Vec<Float>,
    data2: &Vec<Float>,
    final_time: Float,
    dt: Float,
    num_steps: usize,
) {
    // Create a drawing area
    let root_area = BitMapBackend::new("plot 2 sets.png", (800, 600)).into_drawing_area();
    let _ = root_area.fill(&WHITE);

    let min_y = data1
        .iter()
        .chain(data2.iter())
        .fold(Float::INFINITY, |a, &b| a.min(b));

    let max_y = data1
        .iter()
        .chain(data2.iter())
        .fold(Float::NEG_INFINITY, |a, &b| a.max(b));

    // Create chart builder
    let mut chart = ChartBuilder::on(&root_area)
        .caption("X vs. Time", ("sans-serif", 20))
        .margin(10)
        .x_label_area_size(30)
        .y_label_area_size(50)
        .build_cartesian_2d(0.0..final_time, min_y..max_y)
        .unwrap();

    let _ = chart.configure_mesh().draw();

    let _ = chart
        .draw_series(LineSeries::new(
            (0..num_steps).map(|i| (i as Float * dt, data1[i])),
            &RED,
        ))
        .unwrap()
        .label("data 1")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &RED));

    let _ = chart
        .draw_series(LineSeries::new(
            (0..num_steps).map(|i| (i as Float * dt, data2[i])),
            &BLUE,
        ))
        .unwrap()
        .label("data 2")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &BLUE));

    // Configure legend
    let _ = chart
        .configure_series_labels()
        .background_style(&WHITE.mix(0.8))
        .draw();
}

pub fn plot_trajectory(x: &Vec<Float>, y: &Vec<Float>, fname: &str) {
    // Create a drawing backend (e.g., a bitmap)
    let file_name = format!("{}.png", fname);
    let root = BitMapBackend::new(&file_name, (800, 600)).into_drawing_area();
    let _ = root.fill(&WHITE);

    // Determine the range for the plot
    let x_min = *x.iter().min_by(|a, b| a.partial_cmp(b).unwrap()).unwrap();
    let x_max = *x.iter().max_by(|a, b| a.partial_cmp(b).unwrap()).unwrap();
    let y_min = *y.iter().min_by(|a, b| a.partial_cmp(b).unwrap()).unwrap();
    let y_max = *y.iter().max_by(|a, b| a.partial_cmp(b).unwrap()).unwrap();

    // Define the range of the plot
    let mut chart = ChartBuilder::on(&root)
        .caption("2D Trajectory Plot", ("sans-serif", 50).into_font())
        .margin(5)
        .x_label_area_size(30)
        .y_label_area_size(30)
        .build_cartesian_2d(x_min..x_max, y_min..y_max)
        .unwrap();

    // Configure the mesh (grid)
    let _ = chart.configure_mesh().draw();

    // Combine x and y into a vector of tuples
    let trajectory: Vec<(Float, Float)> = x.iter().zip(y.iter()).map(|(&x, &y)| (x, y)).collect();

    // Draw the trajectory
    let _ = chart.draw_series(LineSeries::new(trajectory, &RED));
}
