use std::{
    collections::HashMap,
    ops::{Add, Mul},
};

use itertools::izip;
use nalgebra::{zero, Vector3};

use crate::{
    contact::ContactPoint,
    joint::{Joint, JointVelocity},
    mechanism::MechanismState,
    spatial::spatial_acceleration::SpatialAcceleration,
    spatial::transform::Transform3D,
    types::Float,
};

/// A twist represents the relative angular and linear velocity between two bodies.
/// The twist of frame j with respect to frame i, expressed in frame k is
/// defined as:
///     T_j^(k,i) = (w_j^(k,i)  v_j^(k,i)) \in R^6
/// Twist is a spatial vector.
#[derive(PartialEq, Debug, Clone)]
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
    ///
    /// Given a joint velocity value, it gives out the corresponding twist.
    pub fn new(joint: &Joint, v: &JointVelocity) -> Twist {
        match joint {
            Joint::RevoluteJoint(joint) => Twist {
                body: joint.transform.from.clone(),
                base: joint.transform.to.clone(),
                frame: joint.transform.from.clone(),
                angular: joint.axis.scale(*v.float()),
                linear: zero(),
            },
            Joint::PrismaticJoint(joint) => Twist {
                body: joint.transform.from.clone(),
                base: joint.transform.to.clone(),
                frame: joint.transform.from.clone(),
                angular: zero(),
                linear: joint.axis * *v.float(),
            },
            Joint::FloatingJoint(joint) => Twist {
                body: joint.transform.from.clone(),
                base: joint.transform.to.clone(),
                frame: joint.transform.from.clone(),
                angular: v.spatial().angular,
                linear: v.spatial().linear,
            },
            Joint::FixedJoint(joint) => Twist::zero(&joint.transform.from, &joint.transform.to),
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

        let angular = transform.iso.rotation * self.angular;
        let linear =
            transform.iso.rotation * self.linear + transform.iso.translation.vector.cross(&angular);

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

    /// Compute the velocity of the point that has this twist
    /// TODO: Take a different struct than ContactPoint
    pub fn point_velocity(&self, contact_point: &ContactPoint) -> Vector3<Float> {
        if self.frame != contact_point.frame {
            panic!(
                "Twist frame {} is not equal to contact point frame {}!",
                self.frame, contact_point.frame
            );
        }
        self.linear + self.angular.cross(&contact_point.location)
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

/// Compute the joint twist of each joint expressed in body frame
pub fn compute_joint_twists(state: &MechanismState) -> HashMap<usize, Twist> {
    let mut joint_twists: HashMap<usize, Twist> = HashMap::new();
    for (jointid, joint, v) in izip!(
        state.treejointids.iter(),
        state.treejoints.iter(),
        state.v.iter()
    ) {
        let bodyid = jointid;
        joint_twists.insert(*bodyid, Twist::new(&joint, v));
    }
    joint_twists
}

/// Compute the twist of each body with respect to the world frame, expressed in
/// the world frame
pub fn compute_twists_wrt_world(
    state: &MechanismState,
    bodies_to_root: &HashMap<usize, Transform3D>,
    joint_twists: &HashMap<usize, Twist>,
) -> HashMap<usize, Twist> {
    let mut twists: HashMap<usize, Twist> = HashMap::new();
    let rootid = 0;
    twists.insert(rootid, Twist::zero("world", "world"));
    for jointid in state.treejointids.iter() {
        let parentbodyid = state.parents[*jointid - 1];
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
    use na::{Isometry3, Translation3, UnitQuaternion};
    use nalgebra::vector;

    use crate::{assert_vec_close, PI};

    use super::*;

    #[test]
    fn test_transform1() {
        // Arrange
        let twist_in_body = Twist {
            body: "body".to_string(),
            base: "base".to_string(),
            frame: "body".to_string(),
            angular: vector![0., 1., 0.],
            linear: vector![0., 0., 0.],
        };
        let rot = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), PI/2.0);
        let iso = Isometry3::from_parts(Translation3::new(5., 0., 0.), rot);
        let transform = Transform3D {
            from: "body".to_string(),
            to: "root".to_string(),
            iso
        };

        // Act
        let twist_in_root = twist_in_body.transform(&transform);

        // Assert
        assert_eq!(
            twist_in_root.body, "body"
        );
        assert_eq!(
            twist_in_root.base, "base"
        );
        assert_eq!(
            twist_in_root.frame, "root"
        );
        assert_vec_close!(twist_in_root.angular, vector![0., 0., 1.], 1e-6);
        assert_vec_close!(twist_in_root.linear, vector![0., -5., 0.], 1e-6);
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
        let rot = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), PI/2.0);
        let iso = Isometry3::from_parts(Translation3::new(5., 0., 0.), rot);
        let transform = Transform3D {
            from: "body".to_string(),
            to: "root".to_string(),
            iso,
        };

        // Act
        let twist_in_root = twist_in_body.transform(&transform);

        // Assert
        assert_eq!(
            twist_in_root.body, "body"
        );
        assert_eq!(
            twist_in_root.base, "base"
        );
        assert_eq!(
            twist_in_root.frame, "root"
        );
        assert_vec_close!(twist_in_root.angular, vector![0., 0., 0.], 1e-6);
        assert_vec_close!(twist_in_root.linear, vector![0., 0., 1.], 1e-6);
    }
}
