use std::{collections::HashMap, ops::Mul};

use itertools::izip;
use na::{Isometry, Isometry3, Translation3, UnitQuaternion, UnitVector3};
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
    pub iso: Isometry3<Float> 
}

impl Transform3D {
    pub fn default() -> Self {
        Transform3D {
            from: "body".to_string(),
            to: "world".to_string(),
            iso: Isometry3::identity(),
        }
    }

    pub fn new(from: &str, to: &str, iso: &Isometry3<Float>) -> Self {
        Transform3D {
            from: from.to_string(),
            to: to.to_string(),
            iso: iso.clone(), // TODO: take it rather than cloning it.
        }
    }

    pub fn new_xyz_rpy(from: &str, to:&str, xyz: &Vec<Float>, rpy: &Vec<Float>) ->  Self {
        let x = xyz[0];
        let y = xyz[1];
        let z = xyz[2];
        let translation = Translation3::new(x, y, z);

        let r = rpy[0];
        let p = rpy[1];
        let y = rpy[2];
        let rotation = UnitQuaternion::from_euler_angles(r, p, y);

        let isometry = Isometry3::from_parts(translation, rotation);
        Transform3D {             
            from: from.to_string(),
            to: to.to_string(),
            iso: isometry,
        }
    }

    pub fn identity(from: &str, to: &str) -> Self {
        Transform3D::new(from, to, &Isometry3::identity())
    }

    pub fn move_x(from: &str, to: &str, amount: Float) -> Self {
        Transform3D {
            from: from.to_string(),
            to: to.to_string(),
            iso: Isometry3::translation(amount, 0., 0.)
        }
    }

    pub fn move_z(from: &str, to: &str, amount: Float) -> Self {
        Transform3D {
            from: from.to_string(),
            to: to.to_string(),
            iso: Isometry3::translation(0., 0., amount)
        }
    }

    /// Returns a transformation matrix of translation by amount
    pub fn move_xyz(from: &str, to: &str, x: Float, y: Float, z: Float) -> Self {
        Transform3D {
            from: from.to_string(),
            to: to.to_string(),
            iso: Isometry3::translation(x, y, z)
        }
    }

    pub fn inv(&self) -> Self {
        Transform3D {
            from: self.to.clone(),
            to: self.from.clone(),
            iso: self.iso.inverse(),
        }
    }

    pub fn rot(&self) -> Matrix3<Float> {
        self.iso.rotation.to_rotation_matrix().matrix().into_owned()
    }

    pub fn trans(&self) -> Vector3<Float> {
        self.iso.translation.vector
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
    pub fn rotation(axis: &Vector3<Float>, theta: &Float) -> Matrix4<Float> {
        Isometry3::from_parts(
            Translation3::<Float>::identity(), 
            UnitQuaternion::from_axis_angle(&UnitVector3::new_normalize(*axis), *theta)
        ).to_homogeneous()
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
            iso: self.iso * rhs.iso,
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
            iso: self.iso * rhs.iso,
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

        let axis = Vector3::y_axis(); 
        let treejoints = vec![
            Joint::FloatingJoint(FloatingJoint::new(one_to_world)),
            Joint::RevoluteJoint(RevoluteJoint::new(two_to_one, axis)),
            Joint::RevoluteJoint(RevoluteJoint::new(three_to_two, axis)),
            Joint::RevoluteJoint(RevoluteJoint::new(four_to_one, axis)),
            Joint::RevoluteJoint(RevoluteJoint::new(five_to_four, axis)),
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
        assert_eq!(*two_to_root, Transform3D::new("2", WORLD_FRAME, &Isometry3::translation(-1., 0.,0.)));

        let five_to_root = bodies_to_root.get(&5).unwrap();
        let five_to_root_iso = Isometry3::translation(1., 0., 1.);
        assert_eq!(*five_to_root, Transform3D::new("5", WORLD_FRAME, &five_to_root_iso));
    }
}
