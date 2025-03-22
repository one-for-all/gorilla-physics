use std::{collections::HashMap, ops::Mul};

use itertools::izip;
use nalgebra::{Matrix3, Matrix4, Vector3};

use crate::{mechanism::MechanismState, types::Float};

pub trait Matrix4Ext {
    /// Create the transformation matrix for a linear translation along the x-axis
    fn move_x(amount: Float) -> Matrix4<Float>;

    /// Create the transformation matrix for a linear translation along the y-axis
    fn move_y(amount: Float) -> Matrix4<Float>;

    /// Create the transformation matrix for a linear translation along the z-axis
    fn move_z(amount: Float) -> Matrix4<Float>;
}

impl Matrix4Ext for Matrix4<Float> {
    #[rustfmt::skip]
    fn move_x(amount: Float) -> Matrix4<Float> {
        Matrix4::new(
            1., 0., 0., amount, 
            0., 1., 0., 0., 
            0., 0., 1., 0.,
            0., 0., 0., 1.,
        )
    }

    #[rustfmt::skip]
    fn move_y(amount: Float) -> Matrix4<Float> {
        Matrix4::new(
            1., 0., 0., 0., 
            0., 1., 0., amount, 
            0., 0., 1., 0.,
            0., 0., 0., 1.,
        )
    }

    #[rustfmt::skip]
    fn move_z(amount: Float) -> Matrix4<Float> {
        Matrix4::new(
                1., 0., 0., 0., 
                0., 1., 0., 0., 
                0., 0., 1., amount,
                0., 0., 0., 1.,
        )
    }
}

/// A homogeneous transformation matrix representing the transformation from one
/// 3-dimensional Cartesion coordiante system to another.
#[derive(Debug, PartialEq)]
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

    pub fn move_x(from: &str, to: &str, amount: Float) -> Self {
        Transform3D {
            from: from.to_string(),
            to: to.to_string(),
            mat: Matrix4::<Float>::move_x(amount)
        }
    }

    pub fn move_z(from: &str, to: &str, amount: Float) -> Self {
        Transform3D {
            from: from.to_string(),
            to: to.to_string(),
            mat: Matrix4::<Float>::move_z(amount)
        }
    }

    pub fn inv(&self) -> Self {
        Transform3D {
            from: self.to.clone(),
            to: self.from.clone(),
            mat: self.mat.try_inverse().unwrap(),
        }
    }

    pub fn rot(&self) -> Matrix3<Float> {
        self.mat.fixed_view::<3, 3>(0, 0).into()
    }

    pub fn trans(&self) -> Vector3<Float> {
        self.mat.fixed_view::<3, 1>(0, 3).into()
    }

    pub fn transform_point(&self, point: &Vector3<Float>) -> Vector3<Float> {
        self.rot() * point + self.trans()
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

    /// Returns a transformation matrix from axis-angle
    /// https://en.wikipedia.org/wiki/Rotation_matrix
    #[rustfmt::skip]
    pub fn rotation(axis: &Vector3<Float>, theta: &Float) -> Matrix4<Float> {
        let x = axis.x;
        let y = axis.y;
        let z = axis.z;
        let c = theta.cos();
        let s = theta.sin();
        let t = 1.0 - c;
        Matrix4::new(
            t * x * x + c,      t * x * y - s * z,  t * x * z + s * y,  0.0,
            t * x * y + s * z,  t * y * y + c,      t * y * z - s * x,  0.0,
            t * x * z - s * y,  t * y * z + s * x,  t * z * z + c,      0.0,
            0.0,                0.0,                0.0,                1.0,
        )
    }

    /// Returns a transformation matrix of translation by amount
    #[rustfmt::skip]
    pub fn translation(axis: &Vector3<Float>, distance: &Float) -> Matrix4<Float> {
        let p = axis * (*distance);
        Matrix4::new(
            1.0, 0.0, 0.0, p.x,
            0.0, 1.0, 0.0, p.y,
            0.0, 0.0, 1.0, p.z,
            0.0, 0.0, 0.0, 1.0,
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

pub fn compute_bodies_to_root(state: &MechanismState) -> HashMap<usize, Transform3D> {
    let rootid = 0;
    let mut bodies_to_root = HashMap::new();
    bodies_to_root.insert(rootid, Transform3D::identity("world", "world"));
    for (jointid, joint) in izip!(state.treejointids.iter(), state.treejoints.iter()) {
        let parentbodyid = state.parents[jointid-1];
        let bodyid = jointid;
        let parent_to_root = bodies_to_root.get(&parentbodyid).unwrap();
        let body_to_root = parent_to_root * &joint.transform();
        bodies_to_root.insert(*bodyid, body_to_root);
    }

    bodies_to_root
}

#[cfg(test)]
mod tests {

    use na::vector;
    use crate::{
        joint::{floating::FloatingJoint, revolute::RevoluteJoint, Joint}, rigid_body::RigidBody, WORLD_FRAME
    };

    use super::{compute_bodies_to_root, *};

    #[test]
    #[rustfmt::skip]
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
    #[rustfmt::skip]
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

    /// Verify compute_bodies_to_root fn on the following mechanism
    /// 3         5
    /// |         |
    /// 2 -- 1 -- 4
    #[test]
    fn test_compute_bodies_to_root() {
        // Arrange
        let one_to_world = Transform3D::identity("1", WORLD_FRAME);
        let two_to_one = Transform3D::move_x("2", "1", -1.0);
        let three_to_two = Transform3D::move_z("3", "2", 1.0);
        let four_to_one = Transform3D::move_x("4", "1", 1.0);
        let five_to_four = Transform3D::move_z("5", "4", 1.0);

        let treejoints = vec![
            Joint::FloatingJoint(FloatingJoint::new(one_to_world)),
            Joint::RevoluteJoint(RevoluteJoint::new(two_to_one, vector![0., 1., 0.])),
            Joint::RevoluteJoint(RevoluteJoint::new(three_to_two, vector![0., 1., 0.])),
            Joint::RevoluteJoint(RevoluteJoint::new(four_to_one, vector![0., 1., 0.])),
            Joint::RevoluteJoint(RevoluteJoint::new(five_to_four, vector![0., 1., 0.])),
        ];
        let bodies = vec![
            RigidBody::new_sphere(1., 1., "1"), 
            RigidBody::new_sphere(1., 1., "2"), 
            RigidBody::new_sphere(1., 1., "3"), 
            RigidBody::new_sphere(1., 1., "4"), 
            RigidBody::new_sphere(1., 1., "5")

        ];
        let state = MechanismState::new(treejoints, bodies);

        // Act
        let bodies_to_root = compute_bodies_to_root(&state);

        // Assert
        let one_to_root = bodies_to_root.get(&1).unwrap();
        assert_eq!(*one_to_root, Transform3D::identity("1", WORLD_FRAME));

        let two_to_root = bodies_to_root.get(&2).unwrap();
        assert_eq!(*two_to_root, Transform3D::new("2", WORLD_FRAME, &Matrix4::<Float>::move_x(-1.)));

        let five_to_root = bodies_to_root.get(&5).unwrap();
        let five_to_root_mat = Matrix4::<Float>::move_x(1.) * Matrix4::<Float>::move_z(1.);
        assert_eq!(*five_to_root, Transform3D::new("5", WORLD_FRAME, &five_to_root_mat));
    }
}
