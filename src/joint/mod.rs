use std::ops::{Add, Div, Mul};

use fixed::FixedJoint;
use floating::FloatingJoint;
use na::{dvector, DVector, Vector3};
use prismatic::PrismaticJoint;
use revolute::RevoluteJoint;

use crate::spatial::{
    geometric_jacobian::GeometricJacobian, pose::Pose, spatial_vector::SpatialVector,
    transform::Transform3D,
};
use crate::types::Float;

pub mod constraint_revolute;
pub mod cylindrical_constraint;
pub mod fixed;
pub mod floating;
pub mod prismatic;
pub mod revolute;

pub enum Joint {
    FixedJoint(FixedJoint),
    RevoluteJoint(RevoluteJoint),
    PrismaticJoint(PrismaticJoint),
    FloatingJoint(FloatingJoint),
}

impl Joint {
    pub fn transform(&self) -> &Transform3D {
        match self {
            Joint::FixedJoint(joint) => &joint.transform,
            Joint::RevoluteJoint(joint) => &joint.transform,
            Joint::PrismaticJoint(joint) => &joint.transform,
            Joint::FloatingJoint(joint) => &joint.transform,
        }
    }

    pub fn axis(&self) -> &Vector3<Float> {
        match self {
            Joint::FixedJoint(_) => panic!("Fixed joint has no axis"),
            Joint::RevoluteJoint(joint) => &joint.axis,
            Joint::PrismaticJoint(joint) => &joint.axis,
            Joint::FloatingJoint(_) => panic!("Floating joint has no axis"),
        }
    }

    pub fn motion_subspace(&self) -> GeometricJacobian {
        match self {
            Joint::FixedJoint(joint) => joint.motion_subspace(),
            Joint::RevoluteJoint(joint) => joint.motion_subspace(),
            Joint::PrismaticJoint(joint) => joint.motion_subspace(),
            Joint::FloatingJoint(joint) => joint.motion_subspace(),
        }
    }
}

pub trait ToFloatDVec {
    fn to_float_dvec(&self) -> DVector<Float>;
}

#[derive(Clone, Debug)]
pub enum JointPosition {
    Float(Float), // Single float-valued joint position value
    Pose(Pose),   // 3D pose that represents the position of floating joint
    None,         // for fixed joint
}

impl JointPosition {
    pub fn float(&self) -> &Float {
        match self {
            JointPosition::Float(v) => v,
            _ => panic!("JointPosition is not a float"),
        }
    }

    pub fn pose(&self) -> &Pose {
        match self {
            JointPosition::Pose(pose) => pose,
            _ => panic!("JointPosition is not a pose"),
        }
    }
}

impl ToFloatDVec for Vec<JointPosition> {
    fn to_float_dvec(&self) -> DVector<Float> {
        let mut result: DVector<Float> = dvector![];
        for v in self.iter() {
            match v {
                JointPosition::Float(v) => {
                    result.extend([*v]);
                }
                JointPosition::Pose(v) => {
                    result.extend(v.rotation.coords.iter().cloned()); // [x, y, z, w]
                    result.extend(v.translation.iter().cloned());
                }
                JointPosition::None => {}
            }
        }
        result
    }
}

pub trait ToJointPositionVec {
    fn to_joint_pos_vec(&self) -> Vec<JointPosition>;
}

impl ToJointPositionVec for Vec<Float> {
    fn to_joint_pos_vec(&self) -> Vec<JointPosition> {
        self.iter()
            .map(|f| JointPosition::Float(f.clone()))
            .collect()
    }
}

#[derive(Clone, Debug)]
pub enum JointVelocity {
    Float(Float),           // Single float-valued joint speed value
    Spatial(SpatialVector), // 3D spatial velocity of floating joint
    None,                   // for fixed joint
}

impl JointVelocity {
    pub fn float(&self) -> &Float {
        match self {
            JointVelocity::Float(v) => v,
            _ => panic!("JointVelocity is not a Float"),
        }
    }

    pub fn spatial(&self) -> &SpatialVector {
        match self {
            JointVelocity::Spatial(v) => v,
            _ => panic!("JointVelocity is not a Twist"),
        }
    }
}

impl ToFloatDVec for Vec<JointVelocity> {
    fn to_float_dvec(&self) -> DVector<Float> {
        let mut result: DVector<Float> = dvector![];
        for v in self.iter() {
            match v {
                JointVelocity::Float(v) => {
                    result.extend([*v]);
                }
                JointVelocity::Spatial(v) => {
                    result.extend(v.angular.iter().cloned());
                    result.extend(v.linear.iter().cloned());
                }
                JointVelocity::None => {}
            }
        }
        result
    }
}

pub trait ToJointVelocityVec {
    fn to_joint_vel_vec(&self) -> Vec<JointVelocity>;
}

impl ToJointVelocityVec for Vec<Float> {
    fn to_joint_vel_vec(&self) -> Vec<JointVelocity> {
        self.iter()
            .map(|f| JointVelocity::Float(f.clone()))
            .collect()
    }
}

#[derive(Clone, Debug)]
pub enum JointTorque {
    Float(Float),           // Single float-valued joint torque
    Spatial(SpatialVector), // 3D spatial wrench applied to floating joint
    None,                   // for fixed joint
}

impl JointTorque {
    pub fn float(&self) -> &Float {
        match self {
            JointTorque::Float(v) => v,
            _ => panic!("JointTorque is not a Float"),
        }
    }

    pub fn spatial(&self) -> &SpatialVector {
        match self {
            JointTorque::Spatial(v) => v,
            _ => panic!("JointTorque is not a Twist"),
        }
    }
}

pub trait ToJointTorqueVec {
    fn to_joint_torque_vec(&self) -> Vec<JointTorque>;
}

impl ToJointTorqueVec for Vec<Float> {
    fn to_joint_torque_vec(&self) -> Vec<JointTorque> {
        self.iter().map(|f| JointTorque::Float(f.clone())).collect()
    }
}

impl ToFloatDVec for Vec<JointTorque> {
    fn to_float_dvec(&self) -> DVector<Float> {
        let mut result: DVector<Float> = dvector![];
        for v in self.iter() {
            match v {
                JointTorque::Float(v) => {
                    result.extend([*v]);
                }
                JointTorque::Spatial(v) => {
                    result.extend(v.angular.iter().cloned());
                    result.extend(v.linear.iter().cloned());
                }
                JointTorque::None => {}
            }
        }
        result
    }
}

#[derive(Clone, Debug, Copy)]
pub enum JointAcceleration {
    Float(Float),           // Single float-valued joint acceleration value
    Spatial(SpatialVector), // 3D spatial acceleration of floating joint
    None,                   // for fixed joint
}

impl JointAcceleration {
    pub fn float(&self) -> &Float {
        match self {
            JointAcceleration::Float(v) => v,
            _ => panic!("JointAcceleration is not a Float"),
        }
    }

    pub fn spatial(&self) -> &SpatialVector {
        match self {
            JointAcceleration::Spatial(v) => v,
            _ => panic!("JointAcceleration is not a Spatial Vector"),
        }
    }
}

impl ToFloatDVec for Vec<JointAcceleration> {
    fn to_float_dvec(&self) -> DVector<Float> {
        let mut result: DVector<Float> = dvector![];
        for v in self.iter() {
            match v {
                JointAcceleration::Float(v) => {
                    result.extend([*v]);
                }
                JointAcceleration::Spatial(v) => {
                    result.extend(v.angular.iter().cloned());
                    result.extend(v.linear.iter().cloned());
                }
                JointAcceleration::None => {}
            }
        }
        result
    }
}

impl Mul<Float> for &JointAcceleration {
    type Output = JointAcceleration;

    fn mul(self, rhs: Float) -> Self::Output {
        match self {
            JointAcceleration::Float(ja) => JointAcceleration::Float(ja * rhs),
            JointAcceleration::Spatial(ja) => JointAcceleration::Spatial(ja * rhs),
            JointAcceleration::None => panic!("Fixed joint acceleration None cannot be multiplied"),
        }
    }
}

impl Add for JointAcceleration {
    type Output = JointAcceleration;

    fn add(self, rhs: Self) -> Self::Output {
        match self {
            JointAcceleration::Float(ja) => JointAcceleration::Float(ja + rhs.float()),
            JointAcceleration::Spatial(ja) => JointAcceleration::Spatial(&ja + rhs.spatial()),
            JointAcceleration::None => panic!("Fixed joint acceleration None cannot be added"),
        }
    }
}

impl Div<Float> for JointAcceleration {
    type Output = JointAcceleration;

    fn div(self, rhs: Float) -> Self::Output {
        match self {
            JointAcceleration::Float(ja) => JointAcceleration::Float(ja / rhs),
            JointAcceleration::Spatial(ja) => JointAcceleration::Spatial(&ja / rhs),
            JointAcceleration::None => panic!("Fixed joint acceleration None cannot be divided"),
        }
    }
}
