#[cfg(test)]
mod collision_tests {
    use na::{vector, Vector3};

    use crate::{
        assert_close, assert_vec_close,
        collision::halfspace::HalfSpace,
        hybrid::{articulated::Articulated, visual::Visual, Hybrid, Rigid},
        inertia::SpatialInertia,
        joint::Joint,
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
}
