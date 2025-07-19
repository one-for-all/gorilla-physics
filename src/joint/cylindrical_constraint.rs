use na::{Isometry3, MatrixXx6, UnitVector3, Vector3};

use crate::{joint::constraint_revolute::RevoluteConstraintJoint, types::Float};

pub enum Constraint {
    Revolute(RevoluteConstraintJoint),
    Cylindrical(CylindricalConstraintJoint),
}

impl Constraint {
    pub fn frame1(&self) -> &str {
        match self {
            Self::Revolute(constraint) => constraint.frame1.as_str(),
            Self::Cylindrical(constraint) => constraint.frame1.as_str(),
        }
    }

    pub fn frame2(&self) -> &str {
        match self {
            Self::Revolute(constraint) => constraint.frame2.as_str(),
            Self::Cylindrical(constraint) => constraint.frame2.as_str(),
        }
    }

    pub fn to_frame1(&self) -> &Isometry3<Float> {
        match self {
            Self::Revolute(constraint) => &constraint.to_frame1,
            Self::Cylindrical(constraint) => &constraint.to_frame1,
        }
    }

    pub fn constraint_matrix(&self) -> MatrixXx6<Float> {
        match self {
            Self::Revolute(constraint) => constraint.constraint_matrix(),
            Self::Cylindrical(constraint) => constraint.constraint_matrix(),
        }
    }
}

/// Constrains two frames to only have relative cylindrical motion
pub struct CylindricalConstraintJoint {
    pub axis: UnitVector3<Float>, // axis expressed in the frame after frame 1

    pub frame1: String,
    pub to_frame1: Isometry3<Float>,

    pub frame2: String,
    pub to_frame2: Isometry3<Float>,
}

impl CylindricalConstraintJoint {
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
        let mut matrix = MatrixXx6::zeros(4);

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

        // Sets two remaining rotational axes
        matrix
            .fixed_view_mut::<1, 3>(0, 0)
            .copy_from(&t.transpose());
        matrix
            .fixed_view_mut::<1, 3>(1, 0)
            .copy_from(&b.transpose());

        // Sets two remaining linear axes
        matrix
            .fixed_view_mut::<1, 3>(2, 3)
            .copy_from(&t.transpose());
        matrix
            .fixed_view_mut::<1, 3>(3, 3)
            .copy_from(&b.transpose());

        matrix
    }
}
