use gorilla_physics::{
    contact::{ContactPoint, HalfSpace},
    inertia::SpatialInertia,
    joint::{floating::FloatingJoint, Joint, JointPosition, JointVelocity},
    mechanism::MechanismState,
    pose::Pose,
    rigid_body::RigidBody,
    simulate::step,
    spatial_vector::SpatialVector,
    transform::Transform3D,
    types::Float,
    PI,
};
use nalgebra::{dvector, vector, Matrix3, Rotation3, UnitQuaternion, Vector3};
use plotters::prelude::*;

pub fn main() {
    let m_body = 10.0;
    let r_body = 5.0;

    let l = 10.0;

    let n_foot = 8;

    let moment_x = 2.0 / 5.0 * m_body * r_body * r_body;
    let moment = Matrix3::from_diagonal(&vector![moment_x, moment_x, moment_x]);
    let cross_part = vector![0.0, 0.0, 0.0];

    let body_frame = "body";
    let world_frame = "world";
    let body_to_world = Transform3D::identity(&body_frame, &world_frame);

    let body = RigidBody::new(SpatialInertia {
        frame: body_frame.to_string(),
        mass: m_body,
        moment,
        cross_part,
    });

    let alpha = 2.0 * PI / n_foot as Float / 2.0;

    let treejoints = dvector![Joint::FloatingJoint(FloatingJoint::new(body_to_world))];
    let bodies = dvector![body];
    let halfspace = dvector![];

    let mut state = MechanismState::new(treejoints, bodies, halfspace);
    for i in 0..n_foot {
        let rotation = Rotation3::from_axis_angle(&Vector3::y_axis(), i as Float * 2.0 * alpha);
        let location = rotation * Vector3::new(0., 0., -l);
        state.add_contact_point(&ContactPoint {
            frame: body_frame.to_string(),
            location,
        });
    }

    let h_ground = -20.0;
    let angle: Float = Float::to_radians(10.0);
    let normal = vector![angle.sin(), 0.0, angle.cos()];
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
    let final_time = 50.0;
    let dt = 1.0 / 600.0;
    let num_steps = (final_time / dt) as usize;

    let mut data: Vec<Float> = Vec::with_capacity(num_steps);
    for s in 0..num_steps {
        let torque = vec![];
        let (q, v) = step(&mut state, dt, &torque);

        data.push(v[0].spatial().angular.dot(&Vector3::y_axis()));
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
