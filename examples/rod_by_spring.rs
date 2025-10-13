use gorilla_physics::{
    assert_close,
    collision::halfspace::HalfSpace,
    contact::ContactPoint,
    control::{Controller, NullController},
    inertia::SpatialInertia,
    joint::{
        floating::FloatingJoint,
        prismatic::{JointSpring, PrismaticJoint},
        Joint, JointPosition, JointVelocity,
    },
    mechanism::MechanismState,
    plot::{plot, plot2, plot_trajectory},
    rigid_body::RigidBody,
    simulate::step,
    spatial::{
        pose::Pose,
        spatial_vector::SpatialVector,
        transform::Transform3D,
        twist::{compute_joint_twists, compute_twists_wrt_world},
    },
    types::Float,
    WORLD_FRAME,
};
use nalgebra::{vector, Isometry3, Matrix3, UnitQuaternion, Vector3};

/// Simulate two spheres connected by a spring, hitting ground at an angle.
pub fn main() {
    let m_body = 0.54;
    let r_body = 1.0;
    let m_leg = 1.0;
    let r_leg = 1.0;
    let l_leg = 0.2;
    let l_rest_spring = 0.0;

    let k_spring = 100.0;

    let touch_down_angle = Float::to_radians(-0.0);

    let v_init = 5.0;
    let h_ground = -0.5;

    let alpha = 1.0; // velocity lost
    let mu = 3.5; // coefficient of friction

    let final_time = 1.0;
    let dt = 1.0 / 3600.0;

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
    let axis_leg = -Vector3::z_axis();

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
        Joint::FloatingJoint(FloatingJoint::new(body_to_world)),
        Joint::PrismaticJoint(PrismaticJoint::new_with_spring(
            leg_to_body,
            axis_leg,
            spring,
        )),
    ];

    let bodies = vec![body, leg];
    let mut state = MechanismState::new(treejoints, bodies);

    let ground = HalfSpace::new_with_params(Vector3::z_axis(), h_ground, alpha, mu);
    state.add_halfspace(ground);

    state.add_contact_point(ContactPoint::new(leg_frame, vector![0., 0., 0.]));

    let init_pose = Pose {
        rotation: UnitQuaternion::from_axis_angle(&Vector3::y_axis(), touch_down_angle),
        translation: vector![0.0, 0.0, 0.0],
    };
    let init_vel_linear = init_pose.rotation.inverse() * vector![v_init, 0.0, 0.0];

    let q_init = vec![JointPosition::Pose(init_pose), JointPosition::Float(0.0)];
    let v_init = vec![
        JointVelocity::Spatial(SpatialVector {
            angular: vector![0.0, 0.0, 0.0],
            linear: init_vel_linear,
        }),
        JointVelocity::Float(0.0),
    ];
    state.update(&q_init, &v_init);

    // Simulate
    let num_steps = (final_time / dt) as usize;

    let mut controller = NullController {};

    let mut data1: Vec<Float> = Vec::with_capacity(num_steps);
    let mut data2: Vec<Float> = Vec::with_capacity(num_steps);
    let mut data3: Vec<Float> = Vec::with_capacity(num_steps);
    let mut data4: Vec<Float> = Vec::with_capacity(num_steps);
    let mut data5: Vec<Float> = Vec::with_capacity(num_steps);
    let mut data6: Vec<Float> = Vec::with_capacity(num_steps);
    let mut v_z_prev = 0.0;
    let mut prev_z_leg = 0.0;

    for s in 0..num_steps {
        let torque = controller.control(&mut state, None);
        let (q, v) = step(
            &mut state,
            dt,
            &torque,
            &gorilla_physics::integrators::Integrator::SemiImplicitEuler,
        );

        let rot = q[0].pose().rotation;
        let v_linear = rot * v[0].spatial().linear;
        let _v_angular = rot * v[0].spatial().angular;
        let v_z = v_linear.z;

        let pose_body = &state.poses()[0];
        let pose_leg = &state.poses()[1];

        let bodies_to_root = state.get_bodies_to_root_no_update();
        let joint_twists = compute_joint_twists(&state);
        let twists = compute_twists_wrt_world(&state, &bodies_to_root, &joint_twists);

        let z_leg = pose_leg.translation.z;

        let twist_body = &twists[1];
        let v_body = twist_body.linear + twist_body.angular.cross(&pose_body.translation);

        assert_close!(v_body.z, v_z, 1e-3);

        if prev_z_leg > h_ground && z_leg <= h_ground {
            println!("\n===================\ncontact");
            println!("leg x: {}", pose_leg.translation.x);
            println!("body x: {}", pose_body.translation.x);

            // state.bodies[1].inertia.mass = 200.0;
            // let new_moment_x = 2.0 / 5.0 * 200.0 * r_leg * r_leg;
            // let new_moment_leg =
            //     Matrix3::from_diagonal(&vector![new_moment_x, new_moment_x, new_moment_x]);
            // state.bodies[1].inertia.moment = new_moment_leg;
        }

        if v_z_prev >= 0.0 && v_z < 0.0 {
            println!("===============\napex");
        }

        let _x_body = pose_body.translation.x;
        let _x_leg = pose_leg.translation.x;
        if v_z_prev < 0.0 && v_z >= 0.0 {
            // Bottom
            println!("\n===================\nbottom");
            println!("t: {}", s as Float * dt);
            println!("leg x: {}", pose_leg.translation.x);
            println!("body x: {}", pose_body.translation.x);
        }

        if z_leg > h_ground {
            // let prismatic_jointid = 3;
            // state.set_joint(prismatic_jointid, JointPosition::Float(l_rest_spring));
            // state.set_joint_velocity(prismatic_jointid, JointVelocity::Float(0.0));

            // if let Joint::PrismaticJoint(j) = &mut state.treejoints[2] {
            //     j.spring.as_mut().unwrap().l = l_rest_spring;
            // }
        }

        v_z_prev = v_z;
        prev_z_leg = z_leg;

        let _energy = state.kinetic_energy() + state.gravitational_energy();

        // let body_rotation = q[0].pose().rotation.angle();
        // if body_rotation != 0.0 {
        //     println!("body rotation: {}", body_rotation);
        //     panic!("body rotation is not zero: {}", body_rotation);
        // }

        data1.push(pose_body.translation.x);
        data2.push(pose_body.translation.z);

        // data3.push(pose_leg.translation.x);
        // data4.push(pose_leg.translation.z);

        // data5.push(*q[1].float());
        let twist_leg = &twists[2];
        let v_leg = twist_leg.linear + twist_leg.angular.cross(&pose_leg.translation);
        data5.push(v_leg.x);
        data6.push(pose_leg.translation.z);

        data3.push(pose_leg.translation.x);
        data4.push(pose_body.translation.x);
    }

    plot2(&data3, &data4, final_time, dt, num_steps);
    plot(&data5, final_time, dt, num_steps, "plot");
    plot(&data6, final_time, dt, num_steps, "plot 2");
    plot_trajectory(&data1, &data2, "traj");
    plot_trajectory(&data3, &data4, "traj 2");
}
