use na::{Isometry3, Matrix3, Matrix3xX};

use crate::spatial::{geometric_jacobian::GeometricJacobian, pose::Pose, transform::Transform3D};
use crate::types::Float;

pub struct FloatingJoint {
    pub init_iso: Isometry3<Float>, // initial transform from successor frame to predecessor frame
    pub transform: Transform3D,     // transform from successor frame to predecessor frame
}

impl FloatingJoint {
    pub fn new(transform: Transform3D) -> Self {
        Self {
            init_iso: transform.iso,
            transform,
        }
    }

    /// Update the transform to be intial transform multiplied by pose
    pub fn update(&mut self, q: &Pose) {
        let iso = self.init_iso * q.to_isometry();
        self.transform.iso = iso;
    }

    pub fn motion_subspace(&self) -> GeometricJacobian {
        GeometricJacobian {
            body: self.transform.from.clone(),
            base: self.transform.to.clone(),
            frame: self.transform.from.clone(),
            angular: {
                let mut matrix = Matrix3xX::zeros(6);
                matrix
                    .fixed_view_mut::<3, 3>(0, 0)
                    .copy_from(&Matrix3::identity());
                matrix
            },
            linear: {
                let mut matrix = Matrix3xX::zeros(6);
                matrix
                    .fixed_view_mut::<3, 3>(0, 3)
                    .copy_from(&Matrix3::identity());
                matrix
            },
        }
    }
}

#[cfg(test)]
mod floating_joint_tests {

    use na::{dvector, vector, Matrix3, UnitQuaternion, Vector3};

    use crate::{
        assert_close, assert_vec_close,
        dynamics::dynamics_continuous,
        inertia::SpatialInertia,
        integrators::Integrator,
        joint::{Joint, JointPosition, JointTorque, JointVelocity, Pose, ToFloatDVec},
        mechanism::MechanismState,
        rigid_body::RigidBody,
        simulate::simulate,
        spatial::spatial_vector::SpatialVector,
        GRAVITY, PI, WORLD_FRAME,
    };

    use super::*;

    /// Verify the dynamics of the ball at certain q and v
    #[test]
    fn ball_dynamics() {
        // Arrange
        let m = 5.0;
        let r = 1.0;

        let moment_x = 2.0 / 5.0 * m * r * r;
        let moment_y = 2.0 / 5.0 * m * r * r;
        let moment_z = 2.0 / 5.0 * m * r * r;
        let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
        let cross_part = vector![0.0, 0.0, 0.0];

        let ball_frame = "ball";
        let ball_to_world = Transform3D::identity(&ball_frame, WORLD_FRAME);

        let ball = RigidBody::new(SpatialInertia {
            frame: ball_frame.to_string(),
            moment,
            cross_part,
            mass: m,
        });

        let treejoints = vec![Joint::FloatingJoint(FloatingJoint::new(ball_to_world))];
        let bodies = vec![ball];
        let mut state = MechanismState::new(treejoints, bodies);

        let q_init = vec![JointPosition::Pose(Pose {
            rotation: UnitQuaternion::from_euler_angles(0.1, 0.2, 0.3),
            translation: vector![1.0, 2.0, 3.0],
        })];
        let v_init = vec![JointVelocity::Spatial(SpatialVector {
            angular: vector![1.0, 2.0, 3.0],
            linear: vector![4.0, 5.0, 6.0],
        })];
        state.update(&q_init, &v_init);

        // Act
        let accels = dynamics_continuous(
            &mut state,
            &vec![JointTorque::Spatial(SpatialVector::zero())],
        );

        // Assert
        assert_vec_close!(
            accels.to_float_dvec(),
            dvector![0.0, 0.0, 0.0, 4.948946, -6.959844, -6.566419],
            1e-5
        ); // Reference values from RigidBodyDynamics.jl
    }

    /// Verify that the ball falls under gravity as expected
    #[test]
    fn ball_fall() {
        // Arrange
        let m = 5.0;
        let r = 1.0;

        let moment_x = 2.0 / 5.0 * m * r * r;
        let moment_y = 2.0 / 5.0 * m * r * r;
        let moment_z = 2.0 / 5.0 * m * r * r;
        let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
        let cross_part = vector![0.0, 0.0, 0.0];

        let ball_frame = "ball";
        let ball_to_world = Transform3D::identity(&ball_frame, WORLD_FRAME);

        let ball = RigidBody::new(SpatialInertia {
            frame: ball_frame.to_string(),
            moment,
            cross_part,
            mass: m,
        });

        let treejoints = vec![Joint::FloatingJoint(FloatingJoint::new(ball_to_world))];
        let bodies = vec![ball];
        let mut state = MechanismState::new(treejoints, bodies);

        let height = 1.0;
        let v_z = 1.0;
        let angle = PI / 2.0;
        let angular_v = 1.0;
        let q_init = vec![JointPosition::Pose(Pose {
            rotation: UnitQuaternion::from_axis_angle(&Vector3::x_axis(), angle),
            translation: vector![0.0, 0.0, height],
        })];
        let v_init = vec![JointVelocity::Spatial(SpatialVector {
            angular: vector![angular_v, 0.0, 0.0],
            linear: vector![0.0, v_z, 0.0],
        })];
        state.update(&q_init, &v_init);

        // Act
        let final_time = 5.0;
        let dt = 0.01;
        let (qs, _vs) = simulate(
            &mut state,
            final_time,
            dt,
            |_state| vec![JointTorque::Spatial(SpatialVector::zero())],
            &Integrator::RungeKutta4,
        );

        // Assert
        let q_final = qs.last().unwrap();

        let translation_final = &q_final[0].pose().translation;
        let z_expect = height + v_z * final_time - 0.5 * GRAVITY * final_time * final_time;
        assert_close!(translation_final.x, 0.0, 1e-4);
        assert_close!(translation_final.y, 0.0, 1e-2);
        assert_close!(translation_final.z, z_expect, 1.0);

        let rotation_final = &q_final[0].pose().rotation;
        let rotation_expect =
            UnitQuaternion::from_axis_angle(&Vector3::x_axis(), angle + angular_v * final_time);
        assert_eq!(rotation_final.axis(), rotation_expect.axis());
        assert_close!(rotation_final.angle(), rotation_expect.angle(), 1e-2);
    }
}
