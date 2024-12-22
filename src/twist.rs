use std::{
    collections::HashMap,
    ops::{Add, Mul},
};

use itertools::izip;
use nalgebra::{zero, Vector3};

use crate::{
    joint::RevoluteJoint, mechanism::MechanismState, spatial_acceleration::SpatialAcceleration,
    transform::Transform3D, types::Float,
};

/// A twist represents the relative angular and linear velocity between two bodies.
/// The twist of frame j with respect to frame i, expressed in frame k is
/// defined as:
///     T_j^(k,i) = (w_j^(k,i)  v_j^(k,i)) \in R^6
/// Twist is a spatial vector.
#[derive(PartialEq, Debug)]
pub struct Twist {
    pub body: String,
    pub base: String,
    pub frame: String,
    pub angular: Vector3<Float>,
    pub linear: Vector3<Float>,
}

impl Twist {
    /// Computes the twist of the successor frame wrt. predecessor frame,
    /// expressed in successor frame
    pub fn new(joint: &RevoluteJoint, v: Float) -> Twist {
        Twist {
            body: joint.transform.from.clone(),
            base: joint.transform.to.clone(),
            frame: joint.transform.from.clone(),
            angular: joint.axis * v,
            linear: zero(),
        }
    }

    pub fn zero(body: &str, base: &str) -> Twist {
        Twist {
            body: body.to_string(),
            base: base.to_string(),
            frame: body.to_string(),
            angular: zero(),
            linear: zero(),
        }
    }

    /// Transform the twist to be expressed in the "to" frame of transform
    pub fn transform(&self, transform: &Transform3D) -> Twist {
        if self.frame != transform.from {
            panic!(
                "twist {} frame is not equal to transform `from` {} frame!",
                self.frame, transform.from
            );
        }

        let rot = transform.rot();
        let trans = transform.trans();
        let angular = rot.mul(self.angular);
        let linear = rot.mul(self.linear) + trans.cross(&angular);

        Twist {
            body: self.body.clone(),
            base: self.base.clone(),
            frame: transform.to.clone(),
            angular,
            linear,
        }
    }

    /// Take the spatial cross product of two twists
    /// Returns a spatial acceleration term
    pub fn cross(&self, rhs: &Twist) -> SpatialAcceleration {
        if self.frame != rhs.frame {
            panic!("Frames of two twists do not match!");
        }

        let xw = self.angular;
        let xv = self.linear;
        let yw = rhs.angular;
        let yv = rhs.linear;
        let angular = xw.cross(&yw);
        let linear = xw.cross(&yv) + xv.cross(&yw);

        SpatialAcceleration {
            body: rhs.body.clone(),
            base: rhs.base.clone(),
            frame: self.frame.clone(),
            angular,
            linear,
        }
    }
}

impl<'a, 'b> Add<&'b Twist> for &'a Twist {
    type Output = Twist;

    /// lhs is A to B twist, rhs is B to C twist,
    /// returns A to C twist.
    fn add(self, rhs: &Twist) -> Twist {
        if self.frame != rhs.frame {
            panic!("lhs and rhs are not expressed in the same frame!");
        }

        if self.body != rhs.base {
            panic!("lhs body is not same as rhs base!");
        }

        Twist {
            body: rhs.body.clone(),
            base: self.base.clone(),
            frame: self.frame.clone(),
            angular: self.angular + rhs.angular,
            linear: self.linear + rhs.linear,
        }
    }
}

/// Compute the twist of each body with respect to the world frame, expressed in
/// the world frame
pub fn compute_twists_wrt_world(
    state: &MechanismState,
    bodies_to_root: &HashMap<u32, Transform3D>,
) -> HashMap<u32, Twist> {
    // Compute the twist of each joint expressed in body frame
    let mut joint_twists: HashMap<u32, Twist> = HashMap::new();
    for (jointid, joint, v) in izip!(
        state.treejointids.iter(),
        state.treejoints.iter(),
        state.v.iter()
    ) {
        let bodyid = jointid;
        joint_twists.insert(*bodyid, Twist::new(&joint, *v));
    }

    let mut twists: HashMap<u32, Twist> = HashMap::new();
    let rootid = 0;
    twists.insert(rootid, Twist::zero("world", "world"));
    for jointid in state.treejointids.iter() {
        let parentbodyid = jointid - 1;
        let bodyid = jointid;
        let body_to_root = bodies_to_root.get(&bodyid).unwrap();

        let joint_twist = joint_twists.get(&bodyid).unwrap();
        let parent_twist = twists.get(&parentbodyid).unwrap();

        let body_twist = parent_twist + &joint_twist.transform(body_to_root);
        twists.insert(*bodyid, body_twist);
    }

    twists
}

#[cfg(test)]
#[rustfmt::skip]
mod tests {
    use nalgebra::{vector, Matrix4};

    use super::*;

    #[test]
    fn test_transform() {
        // Arrange
        let twist_in_body = Twist {
            body: "body".to_string(),
            base: "base".to_string(),
            frame: "body".to_string(),
            angular: vector![0., 1., 0.],
            linear: vector![0., 0., 0.],
        };
        let transform = Transform3D {
            from: "body".to_string(),
            to: "root".to_string(),
            mat: Matrix4::new(
                1., 0., 0., 5.,
                0., 0., -1., 0.,
                0., 1., 0., 0.,
                0., 0., 0., 1.,
            ),
        };

        // Act
        let twist_in_root = twist_in_body.transform(&transform);

        // Assert
        assert_eq!(
            twist_in_root,
            Twist {
                body: "body".to_string(),
                base: "base".to_string(),
                frame: "root".to_string(),
                angular: vector![0., 0., 1.],
                linear: vector![0., -5., 0.],
            }
        );
    }

    #[test]
    fn test_transform2() {
        // Arrange
        let twist_in_body = Twist {
            body: "body".to_string(),
            base: "base".to_string(),
            frame: "body".to_string(),
            angular: vector![0., 0., 0.],
            linear: vector![0., 1., 0.],
        };
        let transform = Transform3D {
            from: "body".to_string(),
            to: "root".to_string(),
            mat: Matrix4::new(
                1., 0., 0., 5.,
                0., 0., -1., 0.,
                0., 1., 0., 0.,
                0., 0., 0., 1.,
            ),
        };

        // Act
        let twist_in_root = twist_in_body.transform(&transform);

        // Assert
        assert_eq!(
            twist_in_root,
            Twist {
                body: "body".to_string(),
                base: "base".to_string(),
                frame: "root".to_string(),
                angular: vector![0., 0., 0.],
                linear: vector![0., 0., 1.],
            }
        );
    }
}
