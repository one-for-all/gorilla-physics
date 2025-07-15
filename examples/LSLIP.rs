use gorilla_physics::{
    contact::{ContactPoint, HalfSpace},
    control::Controller,
    inertia::SpatialInertia,
    interface::controller::NullController,
    joint::{
        floating::FloatingJoint,
        prismatic::{JointSpring, PrismaticJoint},
        Joint, JointPosition, JointVelocity,
    },
    mechanism::MechanismState,
    plot::{plot, plot_trajectory},
    rigid_body::RigidBody,
    simulate::step,
    spatial::pose::Pose,
    spatial::spatial_vector::SpatialVector,
    spatial::transform::Transform3D,
    types::Float,
    WORLD_FRAME,
};
use nalgebra::{vector, Isometry3, Matrix3, UnitQuaternion, Vector3};

/// Lossy Spring Loaded Inverted Pendulum (LSLIP)
/// Ref: Nonlinear Model Predictive Control for Rough-Terrain Robot Hopping, 2012
/// ALSLIP without the prismatic actuation
pub fn main() {
    let m_body = 0.54;
    let r_body = 1.0;
    let m_leg = 0.1;
    let r_leg = 1.0;
    let l_leg = 0.2;

    let l_rest_spring = 0.0;
    let k_spring = 500.0;

    // Create hopper body
    let moment_x = 2.0 / 5.0 * m_body * r_body * r_body;
    let moment_body = Matrix3::from_diagonal(&vector![moment_x, moment_x, moment_x]);
    let cross_part_body = vector![0.0, 0.0, 0.0];

    let body_frame = "body";
    let body_to_world = Transform3D::identity(&body_frame, WORLD_FRAME);
    let body = RigidBody::new(SpatialInertia {
        frame: body_frame.to_string(),
        moment: moment_body,
        cross_part: cross_part_body,
        mass: m_body,
    });

    // Create hopper leg
    let moment_x = 2.0 / 5.0 * m_leg * r_leg * r_leg;
    let moment_leg = Matrix3::from_diagonal(&vector![moment_x, moment_x, moment_x]);
    let cross_part_leg = vector![0.0, 0.0, 0.0];
    let axis_leg = vector![0.0, 0.0, -1.0];

    let leg_frame = "leg";
    let leg_to_body = Transform3D {
        from: leg_frame.to_string(),
        to: body_frame.to_string(),
        iso: Isometry3::translation(0., 0., -l_leg),
    };
    let leg = RigidBody::new(SpatialInertia {
        frame: leg_frame.to_string(),
        moment: moment_leg,
        cross_part: cross_part_leg,
        mass: m_leg,
    });

    // Create the hopper
    let spring = JointSpring {
        k: k_spring,
        l: l_rest_spring,
    };
    let treejoints = vec![
        Joint::FloatingJoint(FloatingJoint {
            init_iso: body_to_world.iso,
            transform: body_to_world,
        }),
        Joint::PrismaticJoint(PrismaticJoint::new_with_spring(
            leg_to_body,
            axis_leg,
            spring,
        )),
    ];

    let bodies = vec![body, leg];
    let mut state = MechanismState::new(treejoints, bodies);

    let h_ground = -0.3;
    let alpha = 1.0; // velocity lost
    let mu = 0.5;
    let ground = HalfSpace::new_with_params(Vector3::z_axis(), h_ground, alpha, mu);
    state.add_halfspace(ground);

    state.add_contact_point(ContactPoint::new(leg_frame, vector![0., 0., 0.]));

    let q_init = vec![
        JointPosition::Pose(Pose {
            rotation: UnitQuaternion::identity(),
            translation: vector![0.0, 0.0, 0.0],
        }),
        JointPosition::Float(0.0),
    ];
    let v_init = vec![
        JointVelocity::Spatial(SpatialVector {
            angular: vector![0.0, 0.0, 0.0],
            linear: vector![0.1, 0.0, 0.0],
        }),
        JointVelocity::Float(0.0),
    ];
    state.update(&q_init, &v_init);

    // Simulate
    let final_time = 1.5;
    let dt = 1.0 / 2000.0;
    let num_steps = (final_time / dt) as usize;

    // let mut controller = ALSLIPController::new(k_spring, l_rest_spring);
    let mut controller = NullController {};

    let mut data1: Vec<Float> = Vec::with_capacity(num_steps);
    let mut data2: Vec<Float> = Vec::with_capacity(num_steps);
    let mut data3: Vec<Float> = Vec::with_capacity(num_steps);
    let _v_z_prev = 0.0;

    for _s in 0..num_steps {
        let torque = controller.control(&mut state, None);
        let (_q, _v) = step(
            &mut state,
            dt,
            &torque,
            &gorilla_physics::integrators::Integrator::SemiImplicitEuler,
        );

        let _pose_body = &state.poses()[0]; // q[0].pose();
        let pose_leg = &state.poses()[1];

        data1.push(pose_leg.translation.x);
        data2.push(pose_leg.translation.z);
        data3.push(pose_leg.translation.x);
    }

    plot(&data3, final_time, dt, num_steps, "plot");
    plot_trajectory(&data1, &data2, "traj");
}
