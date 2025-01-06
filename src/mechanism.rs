use std::collections::HashMap;

use crate::geometric_jacobian::GeometricJacobian;
use crate::inertia::compute_inertias;
use crate::inertia::kinetic_energy;
use crate::inertia::SpatialInertia;
use crate::joint::revolute::RevoluteJoint;
use crate::momentum::MomentumMatrix;
use crate::rigid_body::RigidBody;
use crate::transform::compute_bodies_to_root;
use crate::transform::Transform3D;
use crate::twist::compute_twists_wrt_world;
use crate::types::Float;
use itertools::izip;
use na::DMatrix;
use nalgebra::{dvector, DVector};

/// MechanismState stores the state information about the mechanism
pub struct MechanismState {
    pub treejoints: DVector<RevoluteJoint>,
    pub treejointids: DVector<u32>,
    pub bodies: DVector<RigidBody>,
    pub q: DVector<Float>, // joint configuration vector
    pub v: DVector<Float>, // joint velocity vector
}

impl MechanismState {
    pub fn new() -> Self {
        MechanismState {
            treejoints: dvector![RevoluteJoint::default()],
            treejointids: dvector![1],
            bodies: dvector![],
            q: DVector::zeros(0),
            v: DVector::zeros(0),
        }
    }

    pub fn sample() -> Self {
        MechanismState {
            treejoints: dvector![RevoluteJoint::default()],
            treejointids: dvector![1],
            bodies: dvector![RigidBody::default()],
            q: DVector::zeros(0),
            v: DVector::zeros(0),
        }
    }

    pub fn update(&mut self, q: &DVector<Float>, v: &DVector<Float>) {
        self.q = q.clone();
        self.v = v.clone();
        for (joint, q) in izip!(self.treejoints.iter_mut(), q.iter()) {
            joint.update(q);
        }
    }

    /// Computes the total kinetic energy of the system
    pub fn kinetic_energy(&self) -> Float {
        let mut KE = 0.0;
        let bodies_to_root = compute_bodies_to_root(self);
        let twists = compute_twists_wrt_world(self, &bodies_to_root);
        for (jointid, body) in izip!(self.treejointids.iter(), self.bodies.iter()) {
            let bodyid = jointid;
            let twist = twists.get(bodyid).unwrap();
            let body_to_root = bodies_to_root.get(bodyid).unwrap();
            let spatial_inertia = body.inertia.transform(&body_to_root);

            KE += kinetic_energy(&spatial_inertia, &twist);
        }
        KE
    }
}

/// Computes the motion space of each joint, expressed in world frame.
/// We only have revolute joint now, so its motion subspace in body frame is
/// [rotation axis; 0]
pub fn compute_motion_subspaces(
    state: &MechanismState,
    bodies_to_root: &HashMap<u32, Transform3D>,
) -> HashMap<u32, GeometricJacobian> {
    let mut motion_subspaces = HashMap::new();
    for (bodyid, joint) in izip!(state.treejointids.iter(), state.treejoints.iter()) {
        let body_to_root = bodies_to_root.get(bodyid).unwrap();
        // let ms_in_body = GeometricJacobian {
        //     body: body_to_root.from.clone(),
        //     base: "world".to_string(),
        //     frame: body_to_root.from.clone(),
        //     angular: joint.axis,
        //     linear: zero(),
        // };
        let ms_in_body = joint.motion_subspace();
        motion_subspaces.insert(*bodyid, ms_in_body.transform(body_to_root));
    }
    motion_subspaces
}

/// Compute the composite body inertia of each body, expressed in world frame
pub fn compute_crb_inertias(
    state: &MechanismState,
    inertias: &HashMap<u32, SpatialInertia>,
) -> HashMap<u32, SpatialInertia> {
    let mut crb_inertias = HashMap::new();
    for bodyid in state.treejointids.iter().rev() {
        let inertia = inertias.get(bodyid).unwrap();
        let crb_inertia: SpatialInertia;
        if let Some(child_crb_inertia) = crb_inertias.get(&(bodyid + 1)) {
            crb_inertia = inertia + child_crb_inertia;
        } else {
            crb_inertia = inertia.clone();
        }
        crb_inertias.insert(*bodyid, crb_inertia);
    }

    crb_inertias
}

/// Compute the joint-space mass matrix (also known as the inertia matrix) of
/// the Mechanism in the given state, i.e., the matrix M(q) in the unconstrained
/// joint-space equations of motion:
///     M(q) vdot + c(q, v) = τ
/// This method implements the composite rigid body algorithm.
///
/// The result must be a n_v by n_v lower triangular symmetric matrix, where n_v
/// is the dimension of the Mechanism's joint velocity vector v.
pub fn mass_matrix(
    state: &MechanismState,
    bodies_to_root: &HashMap<u32, Transform3D>,
) -> DMatrix<Float> {
    let n_v = state.treejointids.len(); // n_v = number of joints because every joint is a revolute joint
    let mut mass_matrix = DMatrix::zeros(n_v, n_v);
    let motion_subspaces = compute_motion_subspaces(state, bodies_to_root);
    let inertias = compute_inertias(state, bodies_to_root);
    let crb_inertias = compute_crb_inertias(state, &inertias);
    for i in state.treejointids.iter() {
        let Ici = crb_inertias.get(i).unwrap();
        let Si = motion_subspaces.get(i).unwrap();
        let Fi = MomentumMatrix::mul(Ici, Si);
        for j in 1..=*i {
            let Sj = motion_subspaces.get(&j).unwrap();
            let i_indiex: usize = (i - 1).try_into().unwrap();
            let j_index: usize = (j - 1).try_into().unwrap();

            mass_matrix[(i_indiex, j_index)] = Fi.transpose_mul(Sj);
        }
    }

    mass_matrix
}
