use std::collections::HashMap;

use itertools::izip;
use na::{dvector, DVector};
use nalgebra::Vector3;

use crate::{mechanism::MechanismState, transform::Transform3D, types::Float};

/// A wrench represents a system of forces.
/// The wrench w^i expressed in frame i in defined as
///     w^i = (τ^i f^i) = ∑ over j (r_j^i \cross f_j^i   f_j^i)
/// where the f_j^i are forces expressed in frame i, exerted at positions r_j^i.
/// τ^i is the total torque and f^i is the total force.
#[derive(Debug)]
pub struct Wrench {
    pub frame: String,
    pub angular: Vector3<Float>,
    pub linear: Vector3<Float>,
}

/// Compute the torques at each joint that are required to produce the given wrenches.
/// Wrenches are expressed in the world frame.
pub fn compute_torques(
    state: &MechanismState,
    wrenches: &HashMap<u32, Wrench>,
    bodies_to_root: &HashMap<u32, Transform3D>,
) -> DVector<Float> {
    let mut torquesout: DVector<Float> = dvector![];
    for (jointid, joint) in izip!(state.treejointids.iter(), state.treejoints.iter()) {
        let bodyid = jointid;

        let wrench = wrenches.get(bodyid).unwrap(); // TODO: in full impl, should add the wrenches of child bodies
        if wrench.frame != "world" {
            panic!("Wrenches must be expressed in the world frame");
        }

        let body_to_root = bodies_to_root.get(bodyid).unwrap();
        let motion_subspace = joint.motion_subspace().transform(body_to_root);

        let torque = motion_subspace.angular.dot(&wrench.angular)
            + motion_subspace.linear.dot(&wrench.linear);
        torquesout.extend([torque]);
    }

    torquesout
}
