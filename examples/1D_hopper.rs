use gorilla_physics::control::Controller;
use gorilla_physics::joint::floating::FloatingJoint;
use gorilla_physics::joint::prismatic::PrismaticJoint;
use gorilla_physics::joint::Joint;
use gorilla_physics::{
    contact::{ContactPoint, HalfSpace},
    control::energy_control::Hopper1DController,
    energy::hopper_energy,
    inertia::SpatialInertia,
    mechanism::MechanismState,
    plot::plot,
    rigid_body::RigidBody,
    simulate::step,
    spatial::transform::Transform3D,
    types::Float,
};
use nalgebra::{vector, Isometry3, Matrix3, Vector3};

pub fn main() {
    let w_body = 5.0;
    let h_body = 0.1;
    let r_leg = 0.5;
    let r_foot = 0.5;
    let body_leg_length = 2.0;
    let leg_foot_length = 10.0;

    // Create hopper body
    let m_body = 10.0;

    let moment_x = (w_body * w_body + h_body * h_body) * m_body / 12.0;
    let moment_y = (w_body * w_body + h_body * h_body) * m_body / 12.0;
    let moment_z = (w_body * w_body + w_body * w_body) * m_body / 12.0;
    let moment_body = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
    let cross_part_body = vector![0.0, 0.0, 0.0];

    let body_frame = "body";
    let world_frame = "world";
    let body_to_world = Transform3D::identity(&body_frame, &world_frame);
    let body = RigidBody::new(SpatialInertia {
        frame: body_frame.to_string(),
        moment: moment_body,
        cross_part: cross_part_body,
        mass: m_body,
    });

    // Create hopper leg
    let m_leg = 1.0;

    let moment_x = 2.0 / 5.0 * m_leg * r_leg * r_leg;
    let moment_leg = Matrix3::from_diagonal(&vector![moment_x, moment_x, moment_x]);
    let cross_part_leg = vector![0.0, 0.0, 0.0];
    let axis_leg = vector![0.0, 0.0, -1.0];

    let leg_frame = "leg";
    let leg_to_body = Transform3D {
        from: leg_frame.to_string(),
        to: body_frame.to_string(),
        iso: Isometry3::translation(0., 0., -body_leg_length),
    };
    let leg = RigidBody::new(SpatialInertia {
        frame: leg_frame.to_string(),
        moment: moment_leg,
        cross_part: cross_part_leg,
        mass: m_leg,
    });

    // Create hopper foot
    let m_foot = 1.0;

    let moment_x = 2.0 / 5.0 * m_foot * r_foot * r_foot;
    let moment_foot = Matrix3::from_diagonal(&vector![moment_x, moment_x, moment_x]);
    let cross_part_foot = vector![0.0, 0.0, 0.0];
    let axis_foot = vector![0.0, 0.0, -1.0];

    let foot_frame = "foot";
    let foot_to_leg = Transform3D {
        from: foot_frame.to_string(),
        to: leg_frame.to_string(),
        iso: Isometry3::translation(0., 0., -leg_foot_length),
    };
    let foot = RigidBody::new(SpatialInertia {
        frame: foot_frame.to_string(),
        moment: moment_foot,
        cross_part: cross_part_foot,
        mass: m_foot,
    });

    // Create the hopper
    let treejoints = vec![
        Joint::FloatingJoint(FloatingJoint {
            init_iso: body_to_world.iso,
            transform: body_to_world,
        }),
        Joint::PrismaticJoint(PrismaticJoint::new(leg_to_body, axis_leg)),
        Joint::PrismaticJoint(PrismaticJoint::new(foot_to_leg, axis_foot)),
    ];
    let bodies = vec![body, leg, foot];
    let mut state = MechanismState::new(treejoints, bodies);

    state.add_contact_point(ContactPoint::new(body_frame, vector![0., 0., 0.]));

    state.add_contact_point(ContactPoint::new(leg_frame, vector![0., 0., 0.]));

    state.add_contact_point(ContactPoint::new(foot_frame, vector![0., 0., 0.]));

    let ground = HalfSpace::new(Vector3::z_axis(), -20.0);
    state.add_halfspace(ground);

    // Simulate
    let final_time = 30.0;
    let dt = 1.0 / 500.0;
    let num_steps = (final_time / dt) as usize;

    let k_spring = 200.0;
    let mut controller = Hopper1DController {
        k_spring,
        h_setpoint: 0.0,
        leg_length_setpoint: 0.0,
        v_vertical_prev: 0.0,
        body_leg_length,
        leg_foot_length,
    };

    let mut data: Vec<Float> = Vec::with_capacity(num_steps);
    for _ in 0..num_steps {
        let torque = controller.control(&mut state, None);
        let (_q, _v) = step(
            &mut state,
            dt,
            &torque,
            &gorilla_physics::integrators::Integrator::SemiImplicitEuler,
        );
        let _energy = hopper_energy(&state, _q[2].float(), &k_spring);

        data.push(_q[0].pose().translation.z);
    }

    plot(&data, final_time, dt, num_steps, "plot");
}
