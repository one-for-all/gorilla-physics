use na::Matrix3xX;

use crate::spatial::{geometric_jacobian::GeometricJacobian, transform::Transform3D};

pub struct FixedJoint {
    pub transform: Transform3D, // fixed transform from successor frame to predecessor frame
}

impl FixedJoint {
    pub fn new(transform: Transform3D) -> Self {
        Self {
            transform: transform,
        }
    }

    pub fn motion_subspace(&self) -> GeometricJacobian {
        GeometricJacobian {
            body: self.transform.from.clone(),
            base: self.transform.to.clone(),
            frame: self.transform.from.clone(),
            angular: Matrix3xX::zeros(0),
            linear: Matrix3xX::zeros(0),
        }
    }
}

#[cfg(test)]
mod fixed_joint_tests {

    use na::{vector, Isometry3};

    use crate::{
        assert_vec_close,
        dynamics::dynamics_discrete,
        joint::{fixed::FixedJoint, Joint, JointVelocity},
        mechanism::MechanismState,
        rigid_body::RigidBody,
        simulate::step,
        spatial::transform::Transform3D,
        WORLD_FRAME,
    };

    #[test]
    fn fixed_joint_no_velocity() {
        // Arrange
        let frame = "body";
        let body = RigidBody::new_cube(1.0, 1.0, frame);

        let body_to_world = Transform3D::identity(frame, WORLD_FRAME);
        let treejoints = vec![Joint::FixedJoint(FixedJoint::new(body_to_world))];
        let bodies = vec![body];

        let mut state = MechanismState::new(treejoints, bodies);

        // Act
        let v_next = dynamics_discrete(&mut state, &vec![], 1e-1);

        // Assert
        assert_eq!(v_next.len(), 1);
        assert!(matches!(v_next[0], JointVelocity::None));
    }

    #[test]
    fn fixed_joint_no_move() {
        // Arrange
        let frame = "body";
        let body = RigidBody::new_cube(1.0, 1.0, frame);

        let translation = Isometry3::translation(1.0, 0., 0.);
        let rotation = Isometry3::rotation(vector![1., 0., 0.]);
        let init_iso = translation * rotation;
        let body_to_world = Transform3D::new(frame, WORLD_FRAME, &init_iso);
        let treejoints = vec![Joint::FixedJoint(FixedJoint::new(body_to_world))];
        let bodies = vec![body];

        let mut state = MechanismState::new(treejoints, bodies);

        // Act
        let final_time = 1.0;
        let dt = 1e-2;
        let num_steps = (final_time / dt) as usize;
        for _ in 0..num_steps {
            let (_, _) = step(
                &mut state,
                dt,
                &vec![],
                &crate::integrators::Integrator::VelocityStepping,
            );
        }

        // Assert
        let pose = state.poses()[0];
        assert_vec_close!(pose.to_matrix(), init_iso.to_homogeneous(), 1e-6);
    }
}
