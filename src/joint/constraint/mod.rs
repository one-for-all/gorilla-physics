use na::{Isometry3, MatrixXx6};

use crate::{
    joint::constraint::{
        constraint_revolute::RevoluteConstraintJoint,
        cylindrical_constraint::CylindricalConstraintJoint,
    },
    types::Float,
};

pub mod constraint_revolute;
pub mod cylindrical_constraint;

pub enum Constraint {
    Revolute(RevoluteConstraintJoint),
    Cylindrical(CylindricalConstraintJoint),
}

impl Constraint {
    pub fn body1_frame(&self) -> &str {
        match self {
            Self::Revolute(constraint) => constraint.body1_frame.as_str(),
            Self::Cylindrical(constraint) => constraint.body1_frame.as_str(),
        }
    }

    pub fn body2_frame(&self) -> &str {
        match self {
            Self::Revolute(constraint) => constraint.body2_frame.as_str(),
            Self::Cylindrical(constraint) => constraint.body2_frame.as_str(),
        }
    }

    pub fn iso_to_body1(&self) -> &Isometry3<Float> {
        match self {
            Self::Revolute(constraint) => &constraint.to_body1,
            Self::Cylindrical(constraint) => &constraint.to_body1,
        }
    }

    pub fn constraint_matrix(&self) -> MatrixXx6<Float> {
        match self {
            Self::Revolute(constraint) => constraint.constraint_matrix(),
            Self::Cylindrical(constraint) => constraint.constraint_matrix(),
        }
    }
}
