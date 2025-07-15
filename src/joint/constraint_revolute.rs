use na::{Isometry3, Matrix3, MatrixXx6, UnitVector3, Vector3};

use crate::types::Float;

/// Revolute joint that constrains two bodies to only have relative rotation
/// about an axis at a position
/// Note, there are two implicit frames inside the joint. One after frame 1, and
/// one after frame 2. They are not necessarily coincident.
pub struct ConstraintRevoluteJoint {
    pub axis: UnitVector3<Float>, // axis expressed in the frame after frame 1

    pub frame1: String,
    pub to_frame1: Isometry3<Float>,

    pub frame2: String,
    pub to_frame2: Isometry3<Float>,
}

impl ConstraintRevoluteJoint {
    pub fn new(
        frame1: &str,
        to_frame1: Isometry3<Float>,
        frame2: &str,
        to_frame2: Isometry3<Float>,
        axis: UnitVector3<Float>,
    ) -> Self {
        Self {
            axis,
            frame1: frame1.to_string(),
            to_frame1,
            frame2: frame2.to_string(),
            to_frame2,
        }
    }

    /// Matrix that transforms spatial velocity to constraint space
    pub fn constraint_matrix(&self) -> MatrixXx6<Float> {
        let mut matrix = MatrixXx6::zeros(5);

        // Keep linear velocity as is
        matrix
            .fixed_view_mut::<3, 3>(2, 3)
            .copy_from(&Matrix3::identity());

        // Compute two vectors perpendicular to the rotation axis
        let t = {
            let candidate = self.axis.cross(&Vector3::x_axis());
            if candidate.norm() > 1e-3 {
                UnitVector3::new_normalize(candidate)
            } else {
                UnitVector3::new_normalize(self.axis.cross(&Vector3::y_axis()))
            }
        };
        let b = UnitVector3::new_normalize(self.axis.cross(&t));

        matrix
            .fixed_view_mut::<1, 3>(0, 0)
            .copy_from(&t.transpose());
        matrix
            .fixed_view_mut::<1, 3>(1, 0)
            .copy_from(&b.transpose());

        matrix
    }
}

#[cfg(test)]
mod constraint_revolute_tests {
    use crate::{
        assert_vec_close,
        helpers::{build_four_bar_linkage, build_four_bar_linkage_with_base},
        joint::{JointPosition, JointVelocity, ToFloatDVec},
        plot::plot,
        simulate::step,
        types::Float,
        PI,
    };

    use na::dvector;

    #[test]
    fn four_bar_linkage() {
        // Arrange
        let mut state = build_four_bar_linkage(1.0, 1.0);

        // Act
        let final_time = 1.0;
        let dt = 1e-2;
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
        let final_q = state.q;
        assert_vec_close!(final_q.to_float_dvec(), dvector![0., 0., 0.], 1e-7);
    }

    #[test]
    fn four_bar_linkage_heavy() {
        // Arrange
        let mut state = build_four_bar_linkage(1.0, 10.0);
        let angle = PI / 2.0;
        let q = vec![
            JointPosition::Float(-angle),
            JointPosition::Float(-angle),
            JointPosition::Float(angle),
        ];
        state.update_q(&q);

        // Act
        let final_time = 1.0;
        let dt = 1.0 / 60.0;
        let num_steps = (final_time / dt) as usize;
        for s in 0..num_steps {
            let (q, _v) = step(
                &mut state,
                dt,
                &vec![],
                &crate::integrators::Integrator::VelocityStepping,
            );

            // Assert
            assert!(
                !q.to_float_dvec().iter().any(|x| x.is_nan()),
                "simulation exploded at time: {}s",
                s as Float * dt
            );
        }
    }

    #[test]
    fn four_bar_linkage_with_base() {
        // Arrange
        let mut state = build_four_bar_linkage_with_base(1.0, 1.0);
        let angle = PI / 4.0;
        let q = vec![
            JointPosition::Float(0.),
            JointPosition::Float(-angle),
            JointPosition::Float(-angle),
            JointPosition::Float(angle),
        ];
        state.update_q(&q);
        state.set_joint_v(1, JointVelocity::Float(0.1));

        // Act
        let mut data = vec![];
        let final_time = 2.0;
        let dt = 1.0 / 60.0;
        let num_steps = (final_time / dt) as usize;
        for s in 0..num_steps {
            println!("step: {}", s);
            let (q, _v) = step(
                &mut state,
                dt,
                &vec![],
                &crate::integrators::Integrator::VelocityStepping,
            );

            // Assert
            assert!(
                !q.to_float_dvec().iter().any(|x| x.is_nan()),
                "simulation exploded at time: {}s",
                s as Float * dt
            );

            data.push(*q[1].float());
        }

        plot(&data, final_time, dt, num_steps, "4bar with base");
    }
}
