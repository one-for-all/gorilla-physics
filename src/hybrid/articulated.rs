use itertools::izip;
use na::{dvector, vector, DMatrix, DVector, Vector3};

use crate::{
    hybrid::rigid::Rigid,
    inertia::SpatialInertia,
    joint::{Joint, JointPosition, JointVelocity, ToFloatDVec},
    momentum::MomentumMatrix,
    spatial::{
        geometric_jacobian::{Momentum, MotionSubspace},
        pose::Pose,
        spatial_vector::SpatialVector,
        transform::Transform3D,
        wrench,
    },
    types::Float,
    util::{inertia_mul, se3_commutator, spatial_force_cross, spatial_motion_cross},
    GRAVITY,
};

/// bodies connected to each other through joints
pub struct Articulated {
    pub bodies: Vec<Rigid>, // all the rigid bodies in it
    pub joints: Vec<Joint>, // joints[i] connects bodies[i] to its parent

    pub parents: Vec<usize>, // parents[i] -> index of parent body of body i, and parents[i] == i means parent is world
}

impl Articulated {
    pub fn free_velocity(&mut self, dt: Float) -> DVector<Float> {
        // Note: assuming joint iso are up-to-date

        // update body poses
        for (i, joint) in self.joints.iter().enumerate() {
            let parent = self.parents[i];
            assert!(parent <= i); // parents must come before children
            let joint_iso = joint.transform().iso;
            let body_iso = if parent == i {
                // parent is world
                joint_iso
            } else {
                self.bodies[parent].pose.to_isometry() * joint_iso
            };
            self.bodies[i].pose = Pose {
                rotation: body_iso.rotation,
                translation: body_iso.translation.vector,
            }
        }

        // Note: assuming joint twist are up-to-date

        // update body twists
        for (i, joint) in self.joints.iter().enumerate() {
            let parent = self.parents[i];
            let pose = self.bodies[i].pose;
            let joint_twist = joint.twist().transform(&pose);
            let body_twist = if parent == i {
                // parent is world
                joint_twist
            } else {
                self.bodies[parent].twist + joint_twist
            };
            self.bodies[i].twist = body_twist
        }

        // Compute dynamics bias c(q, v)
        // First, compute bias accelerations, i.e. acceleration of each body when no external force,  and no joint acceleration
        let mut bias_accels: Vec<SpatialVector> = vec![];
        for (i, joint) in self.joints.iter().enumerate() {
            let parent = self.parents[i];
            let pose = self.bodies[i].pose;
            let joint_twist = joint.twist().transform(&pose);
            let body_twist = self.bodies[i].twist;
            let coriolis_accel = spatial_motion_cross(&body_twist, &joint_twist);
            let mut bias_accel = if parent == i {
                coriolis_accel
            } else {
                bias_accels[parent] + coriolis_accel
            };
            // add inv gravity accel to simulate gravity force
            bias_accel = bias_accel + SpatialVector::linear(vector![0., 0., GRAVITY]);
            bias_accels.push(bias_accel);
        }

        // Compute the required wrench on each body to achieve the bias accelerations, through Newton-Euler equation
        // ref: equation (5.9) in Featherstone
        let mut wrenches: Vec<SpatialVector> = vec![];
        for (body, accel) in izip!(self.bodies.iter(), bias_accels.iter()) {
            let I = body.inertia_in_world_frame();
            let v = body.twist;

            let Ia = inertia_mul(&I, accel);
            let Iv = inertia_mul(&I, &v);
            let vIv = spatial_force_cross(&v, &Iv);

            wrenches.push(Ia + vIv);
        }

        // Update the wrenches to be wrenches exerted by each joint on the body
        for i in (0..self.joints.len()).rev() {
            let parent = self.parents[i];
            let wrench = wrenches[i];
            if parent != i {
                // parent not world
                wrenches[parent] = wrenches[parent] + wrench;
            }
        }

        // Compute and store motion subspaces, expressed in world frame
        let motion_subspaces: Vec<MotionSubspace> = izip!(self.joints.iter(), self.bodies.iter())
            .map(|(j, b)| j.motion_subspace().as_s().transform(&b.pose))
            .collect();

        // Compute the torques exerted by each joint
        let mut torques: Vec<Float> = vec![];
        for (wrench, motion_subspace) in izip!(wrenches.iter(), motion_subspaces.iter()) {
            let torque: DVector<Float> = motion_subspace.angular.tr_mul(&wrench.angular)
                + motion_subspace.linear.tr_mul(&wrench.linear);
            torques.extend(torque.iter());
        }
        let c = DVector::from_vec(torques); // dynamics bias

        // Compute mass matrix
        let dof = self.dof();
        // compute inertias of each body, expressed in world frame
        let mut inertias: Vec<SpatialInertia> = self
            .bodies
            .iter()
            .map(|b| b.inertia_in_world_frame())
            .collect::<Vec<SpatialInertia>>();
        // update inertias to be composite inertia at each joint
        for i in (0..self.bodies.len()).rev() {
            let parent = self.parents[i];
            if parent != i {
                // not root body
                let (left, right) = inertias.split_at_mut(i);
                left[parent] += &right[0];
            }
        }

        // Compute the starting index of each joint dof
        let dofs: Vec<usize> = self.joints.iter().map(|j| j.dof()).collect();
        let mut indices = vec![];
        let mut i = 0;
        while i < dof {
            indices.push(i);
            let idof = dofs[i]; // ith joint dof
            i += idof;
        }

        // Fill in the mass matrix
        let mut mass_matrix: DMatrix<Float> = DMatrix::zeros(dof, dof);
        for i in 0..self.joints.len() {
            let idof = dofs[i];
            let Ici = &inertias[i];
            let Si = &motion_subspaces[i];
            let Fi = Momentum::mul(Ici, Si);
            let Hii = Fi.transpose_mul(Si);
            let iindex = indices[i];
            mass_matrix
                .view_mut((iindex, iindex), (idof, idof))
                .copy_from(&Hii);

            let mut j = i;
            let mut parent = self.parents[j];
            while parent != j {
                j = parent;
                let jdof = dofs[j];
                let Sj = &motion_subspaces[j];
                let Hij = Fi.transpose_mul(Sj);
                let jindex = indices[j];
                mass_matrix
                    .view_mut((iindex, jindex), (idof, jdof))
                    .copy_from(&Hij);
                mass_matrix
                    .view_mut((jindex, iindex), (jdof, idof))
                    .copy_from(&Hij.transpose());

                parent = self.parents[j];
            }
        }

        let vdot = mass_matrix.cholesky().unwrap().solve(&-c);

        self.v() + vdot * dt
    }

    /// Total degrees of freedom
    fn dof(&self) -> usize {
        self.joints.iter().map(|j| j.dof()).sum()
    }

    /// Velocity vector of the system
    pub fn v(&self) -> DVector<Float> {
        let mut v = vec![];
        for joint in self.joints.iter() {
            v.extend(joint.v().iter());
        }
        DVector::from_vec(v)
    }
}
