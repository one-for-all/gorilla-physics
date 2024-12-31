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
#[derive(Debug, Clone)]
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

    let mut joint_wrenches = (*wrenches).clone();
    joint_wrenches.insert(
        0,
        Wrench {
            frame: "world".to_string(),
            angular: Vector3::zeros(),
            linear: Vector3::zeros(),
        },
    );
    for (jointid, joint) in izip!(
        state.treejointids.iter().rev(),
        state.treejoints.iter().rev()
    ) {
        let bodyid = jointid;

        let joint_wrench = {
            let w = joint_wrenches.get(bodyid).unwrap();
            if w.frame != "world" {
                panic!("Wrenches must be expressed in the world frame");
            }
            w.clone()
        };

        // update parent's joint wrench. action = -reaction
        let parentid = bodyid - 1;
        if let Some(parent_joint_wrench) = joint_wrenches.get_mut(&parentid) {
            parent_joint_wrench.angular += joint_wrench.angular;
            parent_joint_wrench.linear += joint_wrench.linear;
        }

        let body_to_root = bodies_to_root.get(bodyid).unwrap();
        let motion_subspace = joint.motion_subspace().transform(body_to_root);

        let torque = motion_subspace.angular.dot(&joint_wrench.angular)
            + motion_subspace.linear.dot(&joint_wrench.linear);
        torquesout.extend([torque]);
    }
    torquesout.as_mut_slice().reverse(); // Reverse to make it in original order

    torquesout
}
