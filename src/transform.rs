use std::{collections::HashMap, ops::Mul};

use itertools::izip;
use nalgebra::{Matrix3, Matrix4, Vector3};

use crate::{mechanism::MechanismState, types::Float};

/// A homogeneous transformation matrix representing the transformation from one
/// 3-dimensional Cartesion coordiante system to another.
#[derive(Debug)]
pub struct Transform3D {
    pub from: String,
    pub to: String,
    pub mat: Matrix4<Float>,
}

impl Transform3D {
    pub fn default() -> Self {
        Transform3D {
            from: "body".to_string(),
            to: "world".to_string(),
            mat: Matrix4::identity(),
        }
    }

    pub fn new(from: &str, to: &str, mat: &Matrix4<Float>) -> Self {
        Transform3D {
            from: from.to_string(),
            to: to.to_string(),
            mat: mat.clone(),
        }
    }

    pub fn identity(from: &str, to: &str) -> Self {
        Transform3D::new(from, to, &Matrix4::identity())
    }

    pub fn rot(&self) -> Matrix3<Float> {
        self.mat.fixed_view::<3, 3>(0, 0).into()
    }

    pub fn trans(&self) -> Vector3<Float> {
        self.mat.fixed_view::<3, 1>(0, 3).into()
    }


    /// Create the transformation matrix for a rotation about the x-axis
    #[rustfmt::skip]
    pub fn rot_x(theta: Float) -> Matrix4<Float> {
        let c = theta.cos();
        let s = theta.sin();
        Matrix4::new(
                1., 0., 0., 0., 
                0., c, -s, 0., 
                0., s, c, 0.,
                0., 0., 0., 1.,
        )
    }

    /// Create the transformation matrix for a linear translation along the x-axis
    #[rustfmt::skip]
    pub fn move_x(distance: Float) -> Matrix4<Float> {
        Matrix4::new(
                1., 0., 0., distance, 
                0., 1., 0., 0., 
                0., 0., 1., 0.,
                0., 0., 0., 1.,
        )
    }
}

impl Mul for Transform3D {
    type Output = Transform3D;

    fn mul(self, rhs: Self) -> Self::Output {
        if self.from != rhs.to {
            panic!("lhs from frame is not same as rhs to frame!");
        }
        Transform3D {
            from: rhs.from,
            to: self.to,
            mat: self.mat * rhs.mat,
        }
    }
}

impl<'a, 'b> Mul<&'b Transform3D> for &'a Transform3D {
    type Output = Transform3D;
    fn mul(self, rhs: &'b Transform3D) -> Self::Output {
        if self.from != rhs.to {
            panic!("lhs from frame is not same as rhs to frame!");
        }
        Transform3D {
            from: rhs.from.clone(),
            to: self.to.clone(),
            mat: self.mat * rhs.mat,
        }
    }
}

pub fn compute_bodies_to_root(state: &MechanismState) -> HashMap<u32, Transform3D> {
    let rootid = 0;
    let mut bodies_to_root = HashMap::new();
    bodies_to_root.insert(rootid, Transform3D::identity("world", "world"));
    for (jointid, joint) in izip!(state.treejointids.iter(), state.treejoints.iter()) {
        let parentbodyid = jointid - 1;
        let bodyid = jointid;
        let parent_to_root = bodies_to_root.get(&parentbodyid).unwrap();
        let body_to_root = parent_to_root * &joint.transform;
        bodies_to_root.insert(*bodyid, body_to_root);
    }

    bodies_to_root
}

#[cfg(test)]
#[rustfmt::skip]
mod tests {
    use super::*;

    #[test]
    fn test_rot() {
        // Arrange
        let transform = Transform3D {
            from: "a".to_string(),
            to: "b".to_string(),
            mat: Matrix4::new(
                0.0, 0.1, 0.2, 0.3, 
                1.0, 1.1, 1.2, 1.3, 
                2.0, 2.1, 2.2, 2.3, 
                3.0, 3.1, 3.2, 3.3,
            ),
        };

        // Act
        let rot = transform.rot();

        // Assert
        assert_eq!(
            rot,
            Matrix3::new(
                0.0, 0.1, 0.2, 
                1.0, 1.1, 1.2, 
                2.0, 2.1, 2.2)
        )
    }

    #[test]
    fn test_trans() {
        // Arrange
        let transform = Transform3D {
            from: "a".to_string(),
            to: "b".to_string(),
            mat: Matrix4::new(
                0.0, 0.1, 0.2, 0.3, 
                1.0, 1.1, 1.2, 1.3, 
                2.0, 2.1, 2.2, 2.3, 
                3.0, 3.1, 3.2, 3.3,
            ),
        };

        // Act
        let rot = transform.trans();

        // Assert
        assert_eq!(rot, Vector3::new(0.3, 1.3, 2.3))
    }
}