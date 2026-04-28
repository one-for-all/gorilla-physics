#[cfg(test)]
mod collision_tests {
    use na::{vector, Quaternion, UnitQuaternion, Vector3};

    use crate::{
        assert_close, assert_vec_close,
        collision::{halfspace::HalfSpace, sphere},
        hybrid::{articulated::Articulated, visual::Visual, Hybrid, Rigid},
        inertia::SpatialInertia,
        joint::{Joint, JointVelocity},
        spatial::{pose::Pose, spatial_vector::SpatialVector, transform::Transform3D},
        WORLD_FRAME,
    };

    /// collision between a halfspace and an articulated with point contacts
    #[test]
    fn halfspace_articulated_points() {
        // Arrange
        let mut state = Hybrid::empty();

        let halfspace = HalfSpace::new(Vector3::z_axis(), 0.0);
        state.add_halfspace(halfspace);

        let frame = "body";
        let m = 1.;
        let w = 1.;
        let inertia = SpatialInertia::cuboid_at(&Vector3::zeros(), m, w, w, w, frame);
        let mut cube = Rigid::new(inertia);
        // add four point contacts at the bottom
        cube.add_point_at(&vector![w / 2., w / 2., -w / 2.]);
        cube.add_point_at(&vector![w / 2., -w / 2., -w / 2.]);
        cube.add_point_at(&vector![-w / 2., w / 2., -w / 2.]);
        cube.add_point_at(&vector![-w / 2., -w / 2., -w / 2.]);
        let joint = Joint::new_floating(Transform3D::move_z(frame, WORLD_FRAME, w));
        let articulated = Articulated::new(vec![cube], vec![joint]);

        state.add_articulated(articulated);

        // Act
        let final_time = 2.0;
        let dt = 1e-3;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            state.step(dt, &vec![]);
        }

        // Assert
        let pose = state.articulated[0].bodies[0].pose;
        let v = state.articulated[0].v();
        assert_vec_close!(
            pose.as_dvec(),
            Pose::translation(vector![0., 0., w / 2.]).as_dvec(),
            1e-3
        );
        assert_vec_close!(v, SpatialVector::zero().as_dvector(), 1e-3);
    }

    /// collision between a halfspace and an articulated sphere
    #[test]
    fn halfspace_articulated_sphere() {
        // Arrange
        let mut state = Hybrid::empty();

        let halfspace = HalfSpace::new(Vector3::z_axis(), 0.0);
        state.add_halfspace(halfspace);

        let frame = "body";
        let m = 1.;
        let r = 1.;
        let sphere = Rigid::new_sphere(m, r, frame);
        let sphere_joint = Joint::new_floating(Transform3D::move_z(frame, WORLD_FRAME, 2. * r));
        let articulated = Articulated::new(vec![sphere], vec![sphere_joint]);

        state.add_articulated(articulated);

        // Act
        let final_time = 2.0;
        let dt = 1e-3;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            state.step(dt, &vec![]);
        }

        // Assert
        let pose = state.articulated[0].bodies[0].pose;
        let v = state.articulated[0].v();
        assert_vec_close!(
            pose.as_dvec(),
            Pose::translation(vector![0., 0., r]).as_dvec(),
            1e-2
        );
        assert_vec_close!(v, SpatialVector::zero().as_dvector(), 1e-3);
    }

    /// Collision between spheres of two articulated bodies
    #[test]
    fn articulated_spheres() {
        // Arrange
        let mut state = Hybrid::empty();
        state.disable_gravity();

        let m = 1.0;
        let r = 1.0;
        let sphere_frame = "sphere";
        let sphere_body = Rigid::new_sphere(m, r, sphere_frame);
        let sphere_joint = Joint::new_floating(Transform3D::identity(sphere_frame, WORLD_FRAME));
        let sphere = Articulated::new(vec![sphere_body], vec![sphere_joint]);

        let v = -5.0;
        let sphere2_frame = "sphere2";
        let sphere2_body = Rigid::new_sphere(m, r, sphere2_frame);
        let sphere2_joint =
            Joint::new_floating(Transform3D::move_x(sphere2_frame, WORLD_FRAME, 3. * r));
        let mut sphere2 = Articulated::new(vec![sphere2_body], vec![sphere2_joint]);
        sphere2.set_joint_v(
            0,
            JointVelocity::Spatial(SpatialVector::linear(vector![v, 0., 0.])),
        );

        state.add_articulated(sphere);
        state.add_articulated(sphere2);

        // Act
        let final_time = 1.0;
        let dt = 1e-3;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            state.step(dt, &vec![]);
        }

        // Assert
        // Perfectly inelastic collision
        let sphere_pose = state.articulated[0].bodies[0].pose;
        assert!(sphere_pose.translation.x < 0.);
        assert_eq!(sphere_pose.translation.y, 0.);
        assert_eq!(sphere_pose.translation.z, 0.);
        assert_eq!(sphere_pose.rotation, UnitQuaternion::identity());
        let sphere_v = state.articulated[0].v();
        assert!(sphere_v[3] < 0.);

        let sphere2_pose = state.articulated[1].bodies[0].pose;
        assert!(sphere2_pose.translation.x > sphere_pose.translation.x);
        assert_eq!(sphere2_pose.translation.y, 0.);
        assert_eq!(sphere2_pose.translation.z, 0.);
        let sphere2_v = state.articulated[1].v();
        assert_vec_close!(sphere_v, sphere2_v, 1e-3);
    }
}
