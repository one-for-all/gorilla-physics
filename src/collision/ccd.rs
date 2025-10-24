use na::{Matrix2, UnitVector3, Vector2, Vector3};

use crate::{flog, types::Float};

/// Returns (contact point, contact normal, barycentric coords) if vertex collides with face.
pub fn point_face_ccd(
    p: &Vector3<Float>,
    f: &[Vector3<Float>; 3],
    dt: Float,
) -> Option<(Vector3<Float>, UnitVector3<Float>, [Float; 3])> {
    None
}

/// Returns (contact point on edge 1, contact normal, barycentric coords of cp on edge 1 )
/// contact normal is from edge 1 to edge 2
/// TODO: still not fail-proof at super-fast speeds
pub fn edge_edge_ccd(
    e1: &[Vector3<Float>; 2],
    e2: &[Vector3<Float>; 2],
    v1: &Vector3<Float>,
    v2: &Vector3<Float>,
    v3: &Vector3<Float>,
    v4: &Vector3<Float>,
    t_end: Float,
) -> Option<(Vector3<Float>, UnitVector3<Float>, [Float; 2])> {
    let x1 = e1[0];
    let x2 = e1[1];
    let x3 = e2[0];
    let x4 = e2[1];

    // Skip the test if edges are parallel
    if (x4 - x3).cross(&(x2 - x1)).norm() <= 1e-6 {
        return None;
    }

    let a = (v2 - v1).cross(&(v4 - v3));
    let b = (x2 - x1).cross(&(v4 - v3)) + (v2 - v1).cross(&(x4 - x3));
    let c = (x2 - x1).cross(&(x4 - x3));

    let d = x3 - x1;
    let e = v3 - v1;

    let p3 = a.dot(&e);
    let p2 = a.dot(&d) + b.dot(&e);
    let p1 = b.dot(&d) + c.dot(&e);
    let p0 = c.dot(&d);

    let t = solve_cubic(p3, p2, p1, p0);
    if t.is_none() {
        return None;
    }
    let t = t.unwrap();
    if t < 0. || t > t_end {
        return None;
    }

    // ref: Contact and Friction Simulation for Comptuer Graphics, 2022
    // 2.2.3 Continuous Collision Detection of Edge-Edge Pair
    let x1 = x1 + v1 * t;
    let x2 = x2 + v2 * t;
    let x3 = x3 + v3 * t;
    let x4 = x4 + v4 * t;

    let x12 = x2 - x1;
    let x34 = x4 - x3;
    let A00 = x12.dot(&x12);
    let A01 = -x12.dot(&x34);
    let A10 = A01;
    let A11 = x34.dot(&x34);
    let A = Matrix2::new(A00, A01, A10, A11);

    let x13 = x3 - x1;
    let b = Vector2::new(x12.dot(&x13), -x34.dot(&x13));

    if let Some(x) = A.lu().solve(&b) {
        let a = x[0];
        let b = x[1];
        if a > 1. || a < 0. || b > 1. || b < 0. {
            return None;
        }
        let cp = x1 + a * x12;
        let mut n = UnitVector3::new_normalize(x12.cross(&x34));
        if (e2[0] - e1[0]).dot(&n) < 0. {
            n = -n;
        }
        return Some((cp, n, [1. - a, a]));
    } else {
        panic!("no solution on normal equation for edge-edge CCD");
    }
}

/// Cardano's method to solve a cubic equation
/// Solve for x in ax^3 + bx^2 + cx + d = 0
/// ref: https://en.wikipedia.org/wiki/Cubic_equation#Cardano's_formula
pub fn solve_cubic(a: Float, b: Float, c: Float, d: Float) -> Option<Float> {
    if a == 0. {
        return solve_quadratic(b, c, d);
    }

    // Convert to depressed form
    // ref: https://en.wikipedia.org/wiki/Cubic_equation#Depressed_cubic
    let p = (3. * a * c - b * b) / (3. * a * a);
    let q = (2. * b * b * b - 9. * a * b * c + 27. * a * a * d) / (27. * a * a * a);
    let discriminant = (q * q / 4.0) + (p * p * p / 27.0);
    let offset = -b / (3. * a);

    if discriminant > 0.0 {
        // One real root
        let sqrt_disc = discriminant.sqrt();
        let u = (-q / 2. + sqrt_disc).cbrt();
        let v = (-q / 2. - sqrt_disc).cbrt();
        return Some(u + v + offset);
    } else {
        panic!(
            "should not have an equation with non-singular root in edge-edge ccd. discriminant: {}, abcd: {}, {}, {}, {}",
            discriminant, a, b, c, d
        );
    }
}

pub fn solve_quadratic(a: Float, b: Float, c: Float) -> Option<Float> {
    if a == 0. {
        return solve_linear(b, c);
    }

    let det = b * b - 4. * a * c;
    if det < 0. {
        return None;
    }
    if det == 0. {
        return Some(-b / (2. * a));
    } else {
        let sqrt = det.sqrt();
        return Some((-b + sqrt) / (2. * a)); // TODO: there should be two solutions
    }
}

pub fn solve_linear(a: Float, b: Float) -> Option<Float> {
    if a == 0. {
        if b == 0. {
            return Some(0.);
        }
        return None;
    }
    return Some(-b / a);
}

#[cfg(test)]
mod cubic_tests {
    use crate::{assert_close, collision::ccd::solve_cubic};

    #[test]
    fn single_root1() {
        let sol = solve_cubic(1., 0., 1., 1.).unwrap();
        assert_close!(sol, -0.6823278, 1e-5);
    }

    #[test]
    fn single_root2() {
        let sol = solve_cubic(1., 0., -3., 5.).unwrap();
        assert_close!(sol, -2.27902, 1e-5);
    }

    #[test]
    fn single_root3() {
        let sol = solve_cubic(2., 4., 5., 10.).unwrap();
        assert_close!(sol, -2.0, 1e-5);
    }
}

#[cfg(test)]
mod ccd_tests {
    use itertools::izip;
    use na::{vector, UnitQuaternion, UnitVector3, Vector3};
    use rand::{rng, seq::IndexedRandom, Rng};

    use crate::{
        assert_close,
        collision::halfspace::HalfSpace,
        helpers::{build_cube, build_n_spheres, build_sphere},
        joint::{JointPosition, JointVelocity},
        simulate::step,
        spatial::{pose::Pose, spatial_vector::SpatialVector},
        types::Float,
        util::test_utils::random_vector3,
        PI,
    };

    /// Test (sphere - halfspace) CCD
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

    /// Test (contact point - halfspace) CCD in static situation
    #[test]
    fn ccd_cube_ground_stationary() {
        let mut rng = rng();

        let l = 0.1;
        let mut state = build_cube(1.0, l);
        state.add_halfspace(HalfSpace::new(Vector3::z_axis(), 0.));

        let n_tests = 100;
        let size = 100.;
        let tol = 1e-6;
        let angle_choices = [-1.0, -0.5, 0., 0.5, 1.0];
        for _ in 0..n_tests {
            // Arrange
            let x = rng.random_range(-size..size);
            let y = rng.random_range(-size..size);
            let roll = angle_choices.choose(&mut rng).unwrap() * PI;
            let pitch = angle_choices.choose(&mut rng).unwrap() * PI;
            let yaw = angle_choices.choose(&mut rng).unwrap() * PI;
            state.set_joint_q(
                1,
                JointPosition::Pose(Pose {
                    rotation: UnitQuaternion::from_euler_angles(roll, pitch, yaw), // UnitQuaternion::identity(),
                    translation: vector![x, y, l / 2.],
                }),
            );

            // Act
            let final_time = 1.0;
            let dt = 1. / 60.;
            let num_steps = (final_time / dt) as usize;
            for _s in 0..num_steps {
                let (_q, _v) = step(
                    &mut state,
                    dt,
                    &vec![],
                    &crate::integrators::Integrator::CCDVelocityStepping,
                );
            }

            // Assert
            let pos = state.poses()[0].translation;
            let z = pos.z - l / 2.;
            assert_close!(z, 0., tol);
            assert_close!(pos.x, x, tol);
            assert_close!(pos.y, y, tol);
        }
    }

    /// Test (contact point - halfspace) CCD in dynamic situation
    #[test]
    fn ccd_cube_ground_dynamic() {
        let mut rng = rng();

        let l = 0.1;
        let mut state = build_cube(1.0, l);
        state.add_halfspace(HalfSpace::new(Vector3::z_axis(), 0.));

        let n_tests = 100;
        let size = 100.;
        let speed = 1.0; // TODO: can fail when speed to high, or timestep too
                         // large
        let angular_speed = 5.0;
        let tol = 1e-6;
        for _ in 0..n_tests {
            // Arrange
            let x = rng.random_range(-size..size);
            let y = rng.random_range(-size..size);
            let z = rng.random_range(l..(2. * l));
            let axis = UnitVector3::new_normalize(Vector3::new_random());
            let angle = rng.random_range(0.0..(2. * PI));
            state.set_joint_q(
                1,
                JointPosition::Pose(Pose {
                    rotation: UnitQuaternion::from_axis_angle(&axis, angle),
                    // rotation: UnitQuaternion::identity(),
                    translation: vector![x, y, z],
                }),
            );

            let vel = random_vector3(&mut rng, speed);
            let angular = random_vector3(&mut rng, angular_speed);
            state.set_joint_v(
                1,
                JointVelocity::Spatial(SpatialVector {
                    angular,
                    linear: vel,
                }),
            );

            // Act
            let final_time = 1.0;
            let dt = 1. / 60. / 2.0; // TODO: need to have substep of 2, to remain stable w/ current speed limit
            let num_steps = (final_time / dt) as usize;
            for _s in 0..num_steps {
                let (_q, _v) = step(
                    &mut state,
                    dt,
                    &vec![],
                    &crate::integrators::Integrator::CCDVelocityStepping,
                );
            }

            // Assert
            let pos = state.poses()[0].translation;
            let z = pos.z - l / 2.;
            assert_close!(z, 0., tol);
        }
    }
}
