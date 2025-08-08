#[cfg(test)]
mod ccd_tests {
    use na::{vector, UnitQuaternion, Vector3};

    use crate::{
        assert_close,
        contact::HalfSpace,
        helpers::build_sphere,
        joint::{JointPosition, JointVelocity},
        simulate::step,
        spatial::{pose::Pose, spatial_vector::SpatialVector},
    };

    #[ignore]
    #[test]
    fn ccd_sphere_ground() {
        // Arrange
        let radius = 0.1;
        let mut state = build_sphere(1.0, radius);
        state.add_halfspace(HalfSpace::new(Vector3::z_axis(), 0.));

        state.set_joint_q(
            1,
            JointPosition::Pose(Pose {
                rotation: UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.0),
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
        let final_time = 1.0;
        let dt = 1.0 / 60.0;
        let num_steps = (final_time / dt) as usize;
        for _ in 0..num_steps {
            let (_q, _v) = step(
                &mut state,
                dt,
                &vec![],
                &crate::integrators::Integrator::VelocityStepping,
            );
        }

        // Assert
        let z = state.poses()[0].translation.z;
        assert_close!(z, radius, 1e-3);
    }
}
