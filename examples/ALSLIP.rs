use gorilla_physics::{
    contact::{ContactPoint, HalfSpace},
    control::SLIP_control::ALSLIPController,
    inertia::SpatialInertia,
    joint::{
        floating::FloatingJoint,
        prismatic::{JointSpring, PrismaticJoint},
        revolute::RevoluteJoint,
        Joint, JointPosition, JointVelocity,
    },
    mechanism::MechanismState,
    plot::{plot, plot_trajectory},
    rigid_body::RigidBody,
    simulate::step,
    spatial::pose::Pose,
    spatial::spatial_vector::SpatialVector,
    spatial::transform::{compute_bodies_to_root, Transform3D},
    spatial::twist::{compute_joint_twists, compute_twists_wrt_world},
    types::Float,
};
use nalgebra::{vector, Isometry3, Matrix3, UnitQuaternion, Vector3};

/// Actuated Lossy Spring Loaded Inverted Pendulum (ALSLIP)
/// Ref: Nonlinear Model Predictive Control for Rough-Terrain Robot Hopping, 2012
pub fn main() {
    let m_body = 0.54;
    let r_body = 1.0;
    let m_hip = 0.1;
    let r_hip = 1.0;
    let body_hip_length = 0.0;
    let m_leg = 0.1;
    let r_leg = 1.0;
    let l_leg = 0.2;
    let l_rest_spring = 0.0;

    let k_spring = 100.0;

    let touch_down_angle = Float::to_radians(-0.0);
    let v_init = 5.0;
    let h_ground = -0.5;

    let alpha = 2.0; // velocity lost
    let mu = 3.5; // coefficient of friction

    let final_time = 1.5;
    let dt = 1.0 / 3600.0;

    // Create hopper body
    let moment_x = 2.0 / 5.0 * m_body * r_body * r_body;
    let moment_body = Matrix3::from_diagonal(&vector![moment_x, moment_x, moment_x]);
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

    // Create hopper hip
    let moment_x = 2.0 / 5.0 * m_hip * r_hip * r_hip;
    let moment_hip = Matrix3::from_diagonal(&vector![moment_x, moment_x, moment_x]);
    let cross_part_hip = vector![0.0, 0.0, 0.0];
    let axis_hip = Vector3::y_axis();

    let hip_frame = "hip";
    let hip_to_body = Transform3D {
        from: hip_frame.to_string(),
        to: body_frame.to_string(),
        iso: Isometry3::translation(0., 0., -body_hip_length),
    };
    let hip = RigidBody::new(SpatialInertia {
        frame: hip_frame.to_string(),
        moment: moment_hip,
        cross_part: cross_part_hip,
        mass: m_hip,
    });

    // Create hopper leg
    let moment_x = 2.0 / 5.0 * m_leg * r_leg * r_leg;
    let moment_leg = Matrix3::from_diagonal(&vector![moment_x, moment_x, moment_x]);
    let cross_part_leg = vector![0.0, 0.0, 0.0];
    let axis_leg = vector![0., 0., -1.];

    let leg_frame = "leg";
    let leg_to_hip = Transform3D {
        from: leg_frame.to_string(),
        to: hip_frame.to_string(),
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
        Joint::RevoluteJoint(RevoluteJoint::new(hip_to_body, axis_hip)),
        Joint::PrismaticJoint(PrismaticJoint::new_with_spring(
            leg_to_hip, axis_leg, spring,
        )),
    ];

    let bodies = vec![body, hip, leg];
    let mut state = MechanismState::new(treejoints, bodies);

    let ground = HalfSpace::new_with_params(Vector3::z_axis(), h_ground, alpha, mu);
    state.add_halfspace(ground);

    state.add_contact_point(ContactPoint::new(leg_frame, vector![0., 0., 0.]));

    let q_init = vec![
        JointPosition::Pose(Pose {
            rotation: UnitQuaternion::identity(),
            translation: vector![0.0, 0.0, 0.0],
        }),
        JointPosition::Float(touch_down_angle),
        JointPosition::Float(0.0),
    ];
    let v_init = vec![
        JointVelocity::Spatial(SpatialVector {
            angular: vector![0.0, 0.0, 0.0],
            linear: vector![v_init, 0.0, 0.0],
        }),
        JointVelocity::Float(0.0),
        JointVelocity::Float(0.0),
    ];
    state.update(&q_init, &v_init);

    // Simulate
    let num_steps = (final_time / dt) as usize;

    let mut controller = ALSLIPController::new(k_spring, l_rest_spring);

    let mut data1: Vec<Float> = Vec::with_capacity(num_steps);
    let mut data2: Vec<Float> = Vec::with_capacity(num_steps);
    let mut data3: Vec<Float> = Vec::with_capacity(num_steps);
    let mut data4: Vec<Float> = Vec::with_capacity(num_steps);
    let mut data5: Vec<Float> = Vec::with_capacity(num_steps);
    let mut data6: Vec<Float> = Vec::with_capacity(num_steps);
    let mut v_z_prev = 0.0;

    let v_x_integral = 0.0;
    let prev_d_v_z = 0.0;
    let mut prev_z_leg = 0.0;
    for s in 0..num_steps {
        let torque = controller.control(&mut state);
        let (q, v) = step(
            &mut state,
            dt,
            &torque,
            &gorilla_physics::integrators::Integrator::SemiImplicitEuler,
        );

        let rot = q[0].pose().rotation;
        let v_linear = rot * v[0].spatial().linear;
        let v_angular = rot * v[0].spatial().angular;
        let v_z = v_linear.z;
        let v_x = v_linear.x;
        let q_x = q[0].pose().translation.x;
        let q_z = q[0].pose().translation.z;

        let bodies_to_root = compute_bodies_to_root(&state);
        let joint_twists = compute_joint_twists(&state);
        let twists = compute_twists_wrt_world(&state, &bodies_to_root, &joint_twists);

        let pose_body = &state.poses()[0]; // q[0].pose();
        let pose_leg = &state.poses()[2];
        let z_leg = pose_leg.translation.z;
        // println!("leg l: {}", q[2].float());
        if prev_z_leg > h_ground && z_leg <= h_ground {
            println!("\n===================\ncontact");
            println!("leg x: {}", pose_leg.translation.x);
            println!("body x: {}", pose_body.translation.x);
        }

        if v_z_prev >= 0.0 && v_z < 0.0 {
            println!("===============\napex");
            let revolute_jointid = 2;
            let angle = state.q[revolute_jointid - 1].float();
            let degree = Float::to_degrees(*angle);
            // println!("current degree: {}", degree);

            state.set_joint_q(revolute_jointid, JointPosition::Float(touch_down_angle));
            state.set_joint_v(revolute_jointid, JointVelocity::Float(0.0));

            // let mut pose_body = state.poses()[0].clone();
            // pose_body.translation.z += 0.05;
            // state.set_joint(1, JointPosition::Pose(pose_body));

            // let prismatic_jointid = 3;
            // state.set_joint(prismatic_jointid, JointPosition::Float(l_rest_spring));
            // state.set_joint_velocity(prismatic_jointid, JointVelocity::Float(0.0));

            // let mut vel_body = state.v[0].spatial().clone();
            // vel_body.linear.x += 2.0;
            // state.set_joint_velocity(1, JointVelocity::Spatial(vel_body));

            // let q_init = vec![
            //     JointPosition::Pose(Pose {
            //         rotation: UnitQuaternion::identity(),
            //         translation: vector![0.0, 0.0, 0.0],
            //     }),
            //     JointPosition::Float(Float::to_radians(touch_down_angle)),
            //     JointPosition::Float(0.0),
            // ];
            // state.update_q(&q_init);

            // let v_x_target = 0.3;
            // let k1 = 11.0;
            // let k2 = 1.5; // 0.1;
            // let k3 = 1.5; // 5.0;
            // let d_v_x = v_x - v_x_target;
            // v_x_integral += d_v_x;

            // let degree = k1 * d_v_x + k2 * v_x_integral + k3 * (d_v_x - prev_d_v_z);
            // angle = Float::to_radians(degree);

            // prev_d_v_z = d_v_x;
            // let direction = UnitVector3::new_normalize(vector![angle.sin(), 0., -angle.cos()]);

            // state.bodies[0].spring_contacts[0].direction = direction;
            // state.bodies[0].spring_contacts[0].l_rest = l_rest;
        }

        let x_body = pose_body.translation.x;
        let x_leg = pose_leg.translation.x;
        // if v_z_prev < 0.0 && v_z >= 0.0 {
        if v_z_prev < 0.0 && v_z >= 0.0 {
            // Bottom
            // if let Joint::PrismaticJoint(j) = &mut state.treejoints[2] {
            //     j.spring.as_mut().unwrap().l = l_rest_spring + 0.01;
            // }

            println!("\n===================\nbottom");
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

        let revolute_jointid = 2;
        let angle = state.q[revolute_jointid - 1].float();
        let degree = Float::to_degrees(*angle);
        // println!("current degree: {}", degree);

        v_z_prev = v_z;
        prev_z_leg = z_leg;

        // let energy = state.kinetic_energy() + state.gravitational_energy();

        let body_rotation = q[0].pose().rotation.angle();
        // if body_rotation != 0.0 {
        //     println!("body rotation: {}", body_rotation);
        //     panic!("body rotation is not zero: {}", body_rotation);
        // }

        data1.push(pose_body.translation.x);
        data2.push(pose_body.translation.z);
        // data1.push(pose_leg.translation.x);
        // data2.push(pose_leg.translation.z);

        // data1.push(q_x);
        // data2.push(q_z);
        //data3.push(energy);
        data3.push(pose_leg.translation.x);
        data4.push(pose_leg.translation.z);
        // data3.push(*q[2].float());
        // data1.push(v_x);
        // data1.push(theta);
        // data1.push(v[0].spatial().linear.z);
        // data1.push(q[0].pose().translation.x);
        //  data2.push(v[0].spatial().linear.z);
        // println!("{}", q[0].pose().translation.x);

        // println!("{:?}", q[0].pose().rotation);
        // data1.push(q[0].pose().rotation.angle());
        // data1.push(degree);
        // data2.push(v[0].spatial().linear.z);
        // data1.push(energy);
        let twist_leg = twists.get(&3).unwrap();
        let v_leg = twist_leg.linear + twist_leg.angular.cross(&pose_leg.translation);
        data5.push(v_leg.x);
        data6.push(pose_leg.translation.z);
    }

    plot(&data5, final_time, dt, num_steps, "plot");
    plot(&data6, final_time, dt, num_steps, "plot 2");
    plot_trajectory(&data1, &data2, "traj");
    plot_trajectory(&data3, &data4, "traj 2");
}
