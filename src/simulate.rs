use crate::{
    dynamics::newton_euler,
    mechanism::MechanismState,
    spatial_acceleration::SpatialAcceleration,
    spatial_force::compute_torques,
    transform::{compute_bodies_to_root, Transform3D},
    twist::{compute_twists_wrt_world, Twist},
    types::Float,
};
use itertools::izip;
use na::DVector;
use std::collections::HashMap;

pub struct DynamicsResult {}

impl DynamicsResult {
    pub fn new() -> Self {
        DynamicsResult {}
    }
}

pub fn spatial_accelerations(
    state: &mut MechanismState,
    vdot: &DVector<Float>,
) -> (
    HashMap<u32, SpatialAcceleration>,
    HashMap<u32, Transform3D>,
    HashMap<u32, Twist>,
) {
    let joints = &state.treejoints;
    let jointids = &state.treejointids;
    let qs = &state.q;
    let vs = &state.v;
    let vdots = vdot;

    let rootid = 0; // root has body id 0 by convention

    // Compute the body to root frame transform for each body
    let bodies_to_root = compute_bodies_to_root(state);

    // Compute the twist of the each body with respect to the world frame
    let twists = compute_twists_wrt_world(state, &bodies_to_root);

    // Compute the joint spatial accelerations of each body expressed in body frame
    let mut joint_accels: HashMap<u32, SpatialAcceleration> = HashMap::new();
    // joint_accels.insert(rootid, SpatialAcceleration::zero("world", "world"));
    for (jointid, joint, q, v, vdot) in izip!(
        jointids.iter(),
        joints.iter(),
        qs.iter(),
        vs.iter(),
        vdots.iter()
    ) {
        let bodyid = jointid;
        joint_accels.insert(*bodyid, joint.spatial_acceleration(vdot));
    }

    // Compute the spatial acceleration of each body wrt. world frame, expressed
    // in world frame
    let mut accels: HashMap<u32, SpatialAcceleration> = HashMap::new();
    accels.insert(
        rootid,
        SpatialAcceleration::inv_gravitational_spatial_acceleration(),
    ); // simulates the effect of gravity
    for jointid in jointids.iter() {
        let parentbodyid = jointid - 1;
        let bodyid = jointid;
        let parent_acc = accels.get(&parentbodyid).unwrap();
        let parent_twist = twists.get(&parentbodyid).unwrap();
        let body_twist = twists.get(bodyid).unwrap();

        let body_to_root = bodies_to_root.get(bodyid).unwrap();
        let joint_acc = joint_accels.get(bodyid).unwrap().transform(body_to_root);

        let body_acc = &(parent_acc + &joint_acc) + &parent_twist.cross(body_twist);
        accels.insert(*bodyid, body_acc);
    }

    (accels, bodies_to_root, twists)
}

/// Do inverse dynamics, i.e. compute τ in the unconstrained joint-space
/// equations of motion
///
/// M(q) vdot + c(q, v) = τ
///
/// given joint configuration vector q, joint velocity vector v, joint
/// acceleration vector vdot.
///
/// This method implements the recursive Newton-Euler algorithm.
pub fn inverse_dynamics(state: &mut MechanismState, vdot: &DVector<Float>) -> DVector<Float> {
    let (accels, bodies_to_root, twists) = spatial_accelerations(state, &vdot);
    let wrenches = newton_euler(&state, &accels, &bodies_to_root, &twists);
    let torquesout = compute_torques(state, &wrenches, &bodies_to_root);

    torquesout
}

pub fn step(result: &DynamicsResult, state: &MechanismState, t: Float, dt: Float) {}

pub fn integrate(result: &DynamicsResult, state: &MechanismState, final_time: Float, dt: Float) {
    let mut t = 0.0;
    while t < final_time {
        step(&result, &state, t, dt);
        t += dt;
    }
}

pub fn simulate(state: &MechanismState, final_time: Float, dt: Float) {
    let result = DynamicsResult::new();
    integrate(&result, &state, final_time, dt);
}

#[cfg(test)]
mod tests {

    use crate::rigid_body::RigidBody;
    use crate::{inertia::SpatialInertia, joint::RevoluteJoint};

    use super::*;
    use na::Matrix4;
    use nalgebra::{dvector, vector, Matrix3};

    #[test]
    fn inverse_dynamics_upright_rod() {
        // Arrange
        let m = 5.0; // Mass of rod
        let l: Float = 3.0; // Length of rod
        let moment_x = 1.0 / 3.0 * m * l * l;
        let moment_y = moment_x;
        let moment_z = 0.0;
        let cross_part = vector![0.0, 0.0, l / 2.0];

        let mut state = MechanismState {
            treejoints: dvector![RevoluteJoint {
                transform: Transform3D::new("rod", "world", &Matrix4::identity()),
                axis: vector![0.0, 1.0, 0.0]
            }],
            treejointids: dvector![1],
            bodies: dvector![RigidBody {
                inertia: SpatialInertia {
                    frame: "rod".to_string(),
                    moment: Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]),
                    cross_part: cross_part,
                    mass: m
                }
            }],
            q: dvector![0.0],
            v: dvector![0.0],
        };
        let vdot = dvector![-1.0]; // acceleration around -y axis

        // Act
        let torquesout = inverse_dynamics(&mut state, &vdot);

        // Assert
        assert_eq!(torquesout, dvector![-1.0 / 3.0 * m * l * l]);
    }
}