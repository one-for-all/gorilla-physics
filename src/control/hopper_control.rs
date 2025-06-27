//! Control methods for one-legged hoppers

#[cfg(test)]
mod hopper_control_tests {

    use na::{vector, zero, ComplexField, UnitQuaternion, Vector3};

    use crate::{
        assert_close,
        contact::{ContactPoint, HalfSpace},
        control::energy_control::mechanical_stop,
        helpers::build_hopper,
        integrators::Integrator,
        joint::{
            floating::FloatingJoint,
            prismatic::{JointSpring, PrismaticJoint},
            revolute::RevoluteJoint,
            Joint, JointPosition, JointTorque, JointVelocity,
        },
        mechanism::MechanismState,
        plot::plot,
        rigid_body::RigidBody,
        simulate::step,
        spatial::pose::Pose,
        spatial::spatial_vector::SpatialVector,
        spatial::transform::{compute_bodies_to_root, Transform3D},
        spatial::twist::{compute_joint_twists, compute_twists_wrt_world},
        types::Float,
        WORLD_FRAME,
    };

    #[ignore] // TODO: add a control method for ankle-actuated hopper
    #[test]
    fn actuated_ankle_hopper() {
        // Arrange
        let m_foot = 0.5;
        let r_foot = 1.0;
        let m_ankle = 0.5;
        let r_ankle = 1.0;
        let m_body = 10.0;
        let r_body = 1.0;
        let l_ankle_to_body = 1.0;

        let foot_frame = "foot";
        let foot = RigidBody::new_sphere(m_foot, r_foot, &foot_frame);
        let foot_to_world = Transform3D::identity(&foot_frame, WORLD_FRAME);

        let ankle_frame = "ankle";
        let ankle = RigidBody::new_sphere(m_ankle, r_ankle, &ankle_frame);
        let ankle_to_foot = Transform3D::identity(&ankle_frame, &foot_frame);
        let ankle_axis = vector![0., -1., 0.];

        let body_frame = "body";
        let body = RigidBody::new_sphere(m_body, r_body, &body_frame);
        let body_to_ankle = Transform3D::move_z(&body_frame, &ankle_frame, l_ankle_to_body);

        let leg_axis = vector![0., 0., 1.0];
        let leg_spring = JointSpring { k: 1e3, l: 0.0 };

        let treejoints = vec![
            Joint::FloatingJoint(FloatingJoint::new(foot_to_world)),
            Joint::RevoluteJoint(RevoluteJoint::new(ankle_to_foot, ankle_axis)),
            Joint::PrismaticJoint(PrismaticJoint::new_with_spring(
                body_to_ankle,
                leg_axis,
                leg_spring,
            )),
        ];
        let bodies = vec![foot, ankle, body];
        let mut state = MechanismState::new(treejoints, bodies);

        state.add_contact_point(ContactPoint::new(foot_frame, zero()));
        let ground = HalfSpace::new_with_params(Vector3::z_axis(), 0.0, 1.0, 1.0);
        state.add_halfspace(ground);

        state.set_joint_q(
            1,
            JointPosition::Pose(Pose {
                rotation: UnitQuaternion::identity(),
                translation: vector![0., 0., 0.5],
            }),
        );
        state.set_joint_v(
            1,
            JointVelocity::Spatial(SpatialVector {
                angular: zero(),
                linear: vector![0.1, 0.0, 0.0],
            }),
        );

        // Act
        let final_time = 10.0;
        let dt = 1e-3;
        let num_steps = (final_time / dt) as usize;
        let mut data1 = vec![];
        let mut prev_body_vz = 0.0;
        for _ in 0..num_steps {
            let mut torque = vec![
                JointTorque::Spatial(SpatialVector::zero()),
                JointTorque::Float(0.0),
                JointTorque::Float(0.0),
            ];
            if *state.q[2].float() > 0.0 {
                let k_stop = 1e5;
                let b_stop = 125.0;
                let tau_spring = mechanical_stop(
                    0.0,
                    *state.q[2].float(),
                    *state.v[2].float(),
                    k_stop,
                    b_stop,
                );
                torque[2] = JointTorque::Float(tau_spring);
            }

            let (q, v) = step(&mut state, dt, &torque, &Integrator::SemiImplicitEuler);

            let bodies_to_root = compute_bodies_to_root(&state);
            let joint_twists = compute_joint_twists(&state);
            let twists = compute_twists_wrt_world(&state, &bodies_to_root, &joint_twists);
            let body_vz = twists.get(&3).unwrap().linear.z;

            if prev_body_vz >= 0.0 && body_vz < 0.0 {
                println!("peak!");
                let poses = state.poses();
                let pitch = poses[2].rotation.euler_angles().1;
                let ankle_angle = q[1].float();
                state.set_joint_q(2, JointPosition::Float(ankle_angle + pitch));
                state.set_joint_v(2, JointVelocity::Float(0.0));
            }

            if prev_body_vz < 0.0 && body_vz >= 0.0 {
                println!("bottom");
                println!("q2: {}", q[1].float());
                state.set_joint_q(2, JointPosition::Float(-0.42));
            }

            let body_vx = twists.get(&3).unwrap().linear.x;
            data1.push(body_vx);

            prev_body_vz = body_vz;
        }

        // Assert
        plot(&data1, final_time, dt, num_steps, "actuated ankle hopper");
    }

    /// Foot placement algorithm that controls where the hopper goes, but no
    /// regulation on body attitude.
    #[test]
    fn foot_placement_position_control() {
        // Arrange
        let m_foot = 1.0;
        let r_foot = 1.0;
        let m_hip = 0.5;
        let r_hip = 1.0;
        let m_body = 9.5;
        let r_body = 4.0;
        let l_foot_to_hip = 1.0;

        let mut state = build_hopper(m_foot, r_foot, m_hip, r_hip, m_body, r_body, l_foot_to_hip);

        let ground = HalfSpace::new_with_params(Vector3::z_axis(), 0.0, 1.0, 1.0);
        state.add_halfspace(ground);

        let initial_x = 0.5;
        state.set_joint_q(
            1,
            JointPosition::Pose(Pose {
                rotation: UnitQuaternion::identity(),
                translation: vector![initial_x, 0., 0.5],
            }),
        );

        // Act
        let final_time = 10.0;
        let dt = 1e-3;
        let num_steps = (final_time / dt) as usize;
        let mut prev_body_vz = 0.0;
        let mut final_poses = vec![];
        for _ in 0..num_steps {
            let mut torque = vec![
                JointTorque::Spatial(SpatialVector::zero()),
                JointTorque::Float(0.0),
                JointTorque::Float(0.0),
            ];

            // Simulate mechanical stop on the spring
            let spring_q = *state.q[1].float();
            let spring_v = *state.v[1].float();
            if spring_q > 0.0 {
                let k_stop = 1e5;
                let b_stop = 125.0;
                let tau_spring = mechanical_stop(0.0, spring_q, spring_v, k_stop, b_stop);
                torque[1] = JointTorque::Float(tau_spring);
            }

            // body velocity of the body frame, expressed in world frame
            let bodies_to_root = compute_bodies_to_root(&state);
            let joint_twists = compute_joint_twists(&state);
            let twists = compute_twists_wrt_world(&state, &bodies_to_root, &joint_twists);
            let body_twist = twists.get(&3).unwrap();
            let body_vel = body_twist.point_velocity(&ContactPoint::new(
                WORLD_FRAME,
                bodies_to_root.get(&3).unwrap().trans(),
            ));
            let body_vx = body_vel.x;

            // Foot placement
            let poses = state.poses();
            let body_x = poses[2].translation.x;
            let leg_angle = poses[0].rotation.euler_angles().1;
            let leg_angular_v = twists.get(&1).unwrap().angular.y;

            let target_leg_angle = {
                let vx_d = -body_x.signum() * body_x.abs().scale(10.0).min(1.0);
                let x_err = -0.1 * (vx_d - body_vx);

                -((m_body + m_hip + m_foot) * x_err
                    / ((l_foot_to_hip + spring_q) * (m_body + m_hip)))
                    .asin()
            };
            let torque_hip = 2000.0 * (leg_angle - target_leg_angle) + 200.0 * leg_angular_v;
            torque[2] = JointTorque::Float(torque_hip);

            let (q, v) = step(&mut state, dt, &torque, &Integrator::SemiImplicitEuler);

            let bodies_to_root = compute_bodies_to_root(&state);
            let joint_twists = compute_joint_twists(&state);
            let twists = compute_twists_wrt_world(&state, &bodies_to_root, &joint_twists);
            let body_twist = twists.get(&3).unwrap();

            // body velocity of the body frame, expressed in world frame
            let body_vel = body_twist.point_velocity(&ContactPoint::new(
                WORLD_FRAME,
                bodies_to_root.get(&3).unwrap().trans(),
            ));
            let body_vz = body_vel.z;

            // Bottom point
            if prev_body_vz < 0.0 && body_vz >= 0.0 {
                state.set_joint_q(2, JointPosition::Float(-0.42)); // TODO: calculate desired spring compression
            }

            let poses = state.poses();

            prev_body_vz = body_vz;
            final_poses = poses;
        }

        // Assert
        let final_body_pose = &final_poses[2];
        assert_close!(final_body_pose.translation.x, 0.0, 0.05);
    }

    /// Servo attitude control to hop the robot to a specified position
    /// Ref: Hopping in Legged Systems-Modeling and Simulation for the
    /// Two-Dimensional One-Legged Case, Marc Raibert, 1984
    /// TODO: Speed up this test.
    #[test]
    fn servo_attitude_position_control() {
        // Arrange
        let m_foot = 1.0;
        let r_foot = 1.0;
        let m_hip = 0.5;
        let r_hip = 1.0;
        let m_body = 9.5;
        let r_body = 1.0;
        let l_foot_to_hip = 1.0;

        let mut state = build_hopper(m_foot, r_foot, m_hip, r_hip, m_body, r_body, l_foot_to_hip);

        let ground = HalfSpace::new_with_params(Vector3::z_axis(), 0.0, 1.0, 2.0);
        state.add_halfspace(ground);

        // Give the hopper an initial height
        state.set_joint_q(
            1,
            JointPosition::Pose(Pose {
                rotation: UnitQuaternion::identity(),
                translation: vector![-3.0, 0., 0.5],
            }),
        );

        // Act
        let final_time = 20.0;
        let dt = 5e-4;
        let num_steps = (final_time / dt) as usize;
        let mut prev_body_vz = 0.0;
        let mut prev_foot_z = 0.0;

        let mut ground_hit_time = 0.0;
        let mut contact_duration = 0.0;

        let mut final_body_pose = Pose::identity();
        let mut final_body_vel = Vector3::zeros();
        let mut final_body_vel_angular = Vector3::zeros();
        for s in 0..num_steps {
            let mut torque = vec![
                JointTorque::Spatial(SpatialVector::zero()),
                JointTorque::Float(0.0),
                JointTorque::Float(0.0),
            ];

            // Simulate mechanical stop on the spring
            let spring_q = *state.q[1].float();
            let spring_v = *state.v[1].float();
            if spring_q > 0.0 {
                let k_stop = 1e5;
                let b_stop = 125.0;
                let tau_spring = mechanical_stop(0.0, spring_q, spring_v, k_stop, b_stop);
                torque[1] = JointTorque::Float(tau_spring);
            }

            let bodies_to_root = compute_bodies_to_root(&state);
            let joint_twists = compute_joint_twists(&state);
            let twists = compute_twists_wrt_world(&state, &bodies_to_root, &joint_twists);

            let poses = state.poses();
            let foot_z = poses[0].translation.z;

            let current_time = s as Float * dt;
            if prev_foot_z >= 0.0 && foot_z < 0.0 {
                ground_hit_time = current_time;
            }
            if prev_foot_z < 0.0 && foot_z >= 0.0 {
                contact_duration = current_time - ground_hit_time;
            }

            let mut torque_hip = 0.0;
            // Foot placement
            if foot_z >= 0.0 {
                // body velocity of the body frame, expressed in world frame
                let body_twist = twists.get(&3).unwrap();
                let body_vel = body_twist.point_velocity(&ContactPoint::new(
                    WORLD_FRAME,
                    bodies_to_root.get(&3).unwrap().trans(),
                ));
                let body_vx = body_vel.x;

                let body_x = poses[2].translation.x;
                let leg_angle = poses[0].rotation.euler_angles().1;
                let leg_angular_v = twists.get(&1).unwrap().angular.y;
                let w = l_foot_to_hip + spring_q;

                let dvx_max = 0.2;
                let vx_desired = -body_x.signum() * body_x.abs().scale(0.1).min(0.2);
                let dvx = vx_desired - body_vx;
                let x_err = -0.25 * dvx.signum() * dvx.abs().min(dvx_max);

                let x_stance = {
                    if vx_desired < body_vx - dvx_max {
                        body_vx - dvx_max
                    } else if vx_desired > body_vx + dvx_max {
                        body_vx + dvx_max
                    } else {
                        vx_desired
                    }
                } * {
                    if contact_duration == 0.0 {
                        0.4 // estimate of contact duration, when it has no measured value yet
                    } else {
                        contact_duration
                    }
                };

                let x_touchdown =
                    (m_body + m_hip + m_foot) * x_err / (m_body + m_hip) + x_stance / 2.0;
                let target_leg_angle = -(x_touchdown / w).asin();
                torque_hip = 2000.0 * (leg_angle - target_leg_angle) + 200.0 * leg_angular_v;
            }

            // Servo attitude
            if foot_z < 0.0 {
                let body_angle = poses[2].rotation.euler_angles().1;
                let body_angular_v = twists.get(&3).unwrap().angular.y;
                torque_hip = 1200.0 * -body_angle + 60.0 * -body_angular_v;
            }

            torque[2] = JointTorque::Float(torque_hip);

            let (q, v) = step(&mut state, dt, &torque, &Integrator::SemiImplicitEuler);

            let bodies_to_root = compute_bodies_to_root(&state);
            let joint_twists = compute_joint_twists(&state);
            let twists = compute_twists_wrt_world(&state, &bodies_to_root, &joint_twists);
            let body_twist = twists.get(&3).unwrap();

            // body velocity of the body frame, expressed in world frame
            let body_vel = body_twist.point_velocity(&ContactPoint::new(
                WORLD_FRAME,
                bodies_to_root.get(&3).unwrap().trans(),
            ));
            let body_vz = body_vel.z;

            // Bottom point, actuate the spring to jump the robot
            if prev_body_vz < 0.0 && body_vz >= 0.0 {
                state.set_joint_q(2, JointPosition::Float(-0.42)); // TODO: calculate desired spring compression
            }

            prev_body_vz = body_vz;
            prev_foot_z = foot_z;

            final_body_pose = state.poses()[2];
            final_body_vel = body_vel;
            final_body_vel_angular = body_twist.angular;
        }

        // Assert
        assert_close!(final_body_pose.translation.x, 0.0, 1e-1);
        assert_close!(final_body_pose.rotation.euler_angles().1, 0.0, 1e-1);

        assert_close!(final_body_vel.x, 0.0, 1e-1);
        assert_close!(final_body_vel_angular.y, 0.0, 1e-1);
    }
}
