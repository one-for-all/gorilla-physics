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

/// For body1's q1 and body2's q2
/// This constraint keeps q1 to be within [q2+min, q2+max]
pub struct RelativeRangeConstraint {
    pub body1_frame: String,
    pub body2_frame: String,

    pub min: Float,
    pub max: Float,
}

impl RelativeRangeConstraint {
    pub fn new(body1_frame: &str, body2_frame: &str, min: Float, max: Float) -> Self {
        Self {
            body1_frame: body1_frame.to_string(),
            body2_frame: body2_frame.to_string(),
            min,
            max,
        }
    }
}

/// This constraint keeps q to be within [min, max]
pub struct RangeConstraint {
    pub body_frame: String,

    pub min: Float,
    pub max: Float,
}

impl RangeConstraint {
    pub fn new(body_frame: &str, min: Float, max: Float) -> Self {
        Self {
            body_frame: body_frame.to_string(),
            min,
            max,
        }
    }
}
