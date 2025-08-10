#[cfg(test)]
mod ccd_tests {
    use itertools::izip;
    use na::{vector, UnitQuaternion, UnitVector3, Vector3};

    use crate::{
        assert_close,
        contact::HalfSpace,
        helpers::{build_n_spheres, build_sphere},
        joint::{JointPosition, JointVelocity},
        simulate::step,
        spatial::{pose::Pose, spatial_vector::SpatialVector},
        types::Float,
    };

    #[test]
    fn ccd_sphere_ground() {
        // Arrange
        let radius = 0.1;
        let mut state = build_sphere(1.0, radius);
        state.add_halfspace(HalfSpace::new(Vector3::z_axis(), 0.));

        state.set_joint_q(
            1,
            JointPosition::Pose(Pose {
                rotation: UnitQuaternion::identity(),
                translation: vector![0.0, 0.0, 10. * radius],
            }),
        );
        state.set_joint_v(
            1,
            JointVelocity::Spatial(SpatialVector {
                angular: vector![0.0, 0.0, 0.0],
                linear: vector![0.0, 0.0, -5.0],
            }),
        );

        // Act
        let mut max_penetration = 0.;
        let final_time = 1.0;
        let dt = 1.0 / 60.0;
        let num_steps = (final_time / dt) as usize;
        for _ in 0..num_steps {
            let (_q, _v) = step(
                &mut state,
                dt,
                &vec![],
                &crate::integrators::Integrator::CCDVelocityStepping,
            );

            let z = state.poses()[0].translation.z;
            let penetration = (radius - z).max(0.);
            if penetration > max_penetration {
                max_penetration = penetration;
            }
        }

        // Assert
        // steady state
        let z = state.poses()[0].translation.z;
        assert_close!(z, radius, 1e-8);

        // max penetration
        assert!(max_penetration < 1e-8);
    }

    #[test]
    fn ccd_sphere_tilted_ground() {
        // Arrange
        let radius = 0.1;
        let mut state = build_sphere(1.0, radius);

        let angle = Float::to_radians(10.0);
        let normal = UnitVector3::new_normalize(vector![angle.sin(), 0., angle.cos()]);
        state.add_halfspace(HalfSpace::new(normal, 0.));

        state.set_joint_q(
            1,
            JointPosition::Pose(Pose {
                rotation: UnitQuaternion::identity(),
                translation: vector![0.0, 0.0, 10. * radius],
            }),
        );
        state.set_joint_v(
            1,
            JointVelocity::Spatial(SpatialVector {
                angular: vector![0.0, 0.0, 0.0],
                linear: vector![0.0, 0.0, -5.0],
            }),
        );

        // Act
        let mut max_penetration = 0.;
        let final_time = 0.5;
        let dt = 1.0 / 60.0;
        let num_steps = (final_time / dt) as usize;
        for _ in 0..num_steps {
            let (_q, _v) = step(
                &mut state,
                dt,
                &vec![],
                &crate::integrators::Integrator::CCDVelocityStepping,
            );

            let distance = state.bodies[0]
                .collider
                .as_ref()
                .unwrap()
                .geometry
                .sphere()
                .distance_halfspace(&state.halfspaces[0]);
            let penetration = (-distance).max(0.);
            if penetration > max_penetration {
                max_penetration = penetration;
            }
        }

        // Assert
        // max penetration
        assert!(max_penetration < 1e-8);

        // tangential speed
        let twists = state.get_body_twists();
        let twist = &twists[1];
        let center = state.poses()[0].translation;
        let v = twist.point_velocity(&center);
        assert!(v.x > 1.0);
        assert_close!(v.y, 0., 1e-5);
    }

    #[test]
    fn ccd_three_sphere_ground() {
        // Arrange
        let radius = 0.1;
        let mut state = build_n_spheres(1.0, radius, 3);

        let angle = Float::to_radians(0.0);
        let normal = UnitVector3::new_normalize(vector![angle.sin(), 0., angle.cos()]);
        state.add_halfspace(HalfSpace::new(normal, 0.));

        let ys = vec![0., 1., 2.];

        let q_init = vec![
            JointPosition::Pose(Pose {
                rotation: UnitQuaternion::identity(),
                translation: vector![0.0, ys[0], 10. * radius],
            }),
            JointPosition::Pose(Pose {
                rotation: UnitQuaternion::identity(),
                translation: vector![0.0, ys[1], 10. * radius + 1e-8],
            }),
            JointPosition::Pose(Pose {
                rotation: UnitQuaternion::identity(),
                translation: vector![0.0, ys[2], 10. * radius + 2e-8],
            }),
        ];
        let v_init = vec![
            JointVelocity::Spatial(SpatialVector {
                angular: vector![0.0, 0.0, 0.0],
                linear: vector![0.0, 0.0, -5.0],
            });
            3
        ];
        state.update(&q_init, &v_init);

        // Act
        let mut max_penetration = 0.;
        let final_time = 1.0;
        let dt = 1.0 / 60.0;
        let num_steps = (final_time / dt) as usize;
        for _ in 0..num_steps {
            let (_q, _v) = step(
                &mut state,
                dt,
                &vec![],
                &crate::integrators::Integrator::CCDVelocityStepping,
            );

            for body in state.bodies.iter() {
                let distance = body
                    .collider
                    .as_ref()
                    .unwrap()
                    .geometry
                    .sphere()
                    .distance_halfspace(&state.halfspaces[0]);
                let penetration = (-distance).max(0.);
                if penetration > max_penetration {
                    max_penetration = penetration;
                }
            }
        }

        // Assert
        // steady state
        for (y_init, pose) in izip!(ys.iter(), state.poses().iter()) {
            assert_close!(pose.translation.z, radius, 1e-8);
            assert_eq!(pose.translation.x, 0.);
            assert_eq!(pose.translation.y, *y_init);
        }

        // max penetration
        assert!(max_penetration < 1e-8);
    }
}
