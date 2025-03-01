use std::collections::HashMap;

use crate::contact::ContactPoint;
use crate::contact::HalfSpace;
use crate::contact::SpringContact;
use crate::geometric_jacobian::GeometricJacobian;
use crate::inertia::compute_inertias;
use crate::inertia::kinetic_energy;
use crate::inertia::SpatialInertia;
use crate::joint::Joint;
use crate::joint::JointPosition;
use crate::joint::JointVelocity;
use crate::momentum::MomentumMatrix;
use crate::pose::Pose;
use crate::rigid_body::RigidBody;
use crate::spatial_vector::SpatialVector;
use crate::transform::compute_bodies_to_root;
use crate::transform::Transform3D;
use crate::twist::compute_joint_twists;
use crate::twist::compute_twists_wrt_world;
use crate::types::Float;
use crate::GRAVITY;
use itertools::izip;
use na::dvector;
use na::DMatrix;
use nalgebra::DVector;

/// MechanismState stores the state information about the mechanism
pub struct MechanismState {
    pub treejoints: DVector<Joint>,
    pub treejointids: DVector<usize>,
    pub bodies: DVector<RigidBody>,
    pub q: Vec<JointPosition>, // joint configuration/position vector
    pub v: Vec<JointVelocity>, // joint velocity vector
    pub halfspaces: DVector<HalfSpace>,
}

impl MechanismState {
    /// Create a new MechanismState with zero initial condition
    pub fn new(
        treejoints: DVector<Joint>,
        bodies: DVector<RigidBody>,
        halfspaces: DVector<HalfSpace>,
    ) -> Self {
        let njoints = treejoints.len();
        let mut q = vec![];
        let mut v = vec![];
        for j in treejoints.iter() {
            match j {
                Joint::RevoluteJoint(_) => {
                    q.push(JointPosition::Float(0.0));
                    v.push(JointVelocity::Float(0.0));
                }
                Joint::PrismaticJoint(_) => {
                    q.push(JointPosition::Float(0.0));
                    v.push(JointVelocity::Float(0.0));
                }
                Joint::FloatingJoint(_) => {
                    q.push(JointPosition::Pose(Pose::identity()));
                    v.push(JointVelocity::Spatial(SpatialVector::zero()));
                }
            }
        }

        MechanismState {
            treejoints,
            treejointids: DVector::from_vec(Vec::from_iter(1..njoints + 1)),
            bodies,
            q,
            v,
            halfspaces,
        }
    }

    pub fn update(&mut self, q: &Vec<JointPosition>, v: &Vec<JointVelocity>) {
        self.update_q(q);
        self.v = v.clone();
    }

    pub fn update_q(&mut self, q: &Vec<JointPosition>) {
        self.q = q.clone();
        for (joint, q) in izip!(self.treejoints.iter_mut(), q.iter()) {
            match joint {
                Joint::RevoluteJoint(j) => {
                    if let JointPosition::Float(q) = q {
                        j.update(q)
                    } else {
                        panic!("Revolute joint expects a Float position");
                    }
                }
                Joint::PrismaticJoint(j) => {
                    if let JointPosition::Float(q) = q {
                        j.update(q)
                    } else {
                        panic!("Prismatic joint expects a Float position");
                    }
                }
                Joint::FloatingJoint(j) => {
                    if let JointPosition::Pose(q) = q {
                        j.update(q)
                    } else {
                        panic!("Floating joint expects a Pose position");
                    }
                }
            }
        }
    }

    /// Computes the total kinetic energy of the system
    pub fn kinetic_energy(&self) -> Float {
        let mut KE = 0.0;
        let bodies_to_root = compute_bodies_to_root(self);
        let joint_twists = compute_joint_twists(self);
        let twists = compute_twists_wrt_world(self, &bodies_to_root, &joint_twists);
        for (jointid, body) in izip!(self.treejointids.iter(), self.bodies.iter()) {
            let bodyid = jointid;
            let twist = twists.get(bodyid).unwrap();
            let body_to_root = bodies_to_root.get(bodyid).unwrap();
            let spatial_inertia = body.inertia.transform(&body_to_root);

            let ke = kinetic_energy(&spatial_inertia, &twist);
            KE += ke;
        }
        KE
    }

    pub fn gravitational_energy(&self) -> Float {
        let mut PE = 0.0;
        let bodies_to_root = compute_bodies_to_root(self);
        for jointid in self.treejointids.iter() {
            let height = bodies_to_root.get(jointid).unwrap().trans().z;
            let mass = self.bodies[jointid - 1].inertia.mass;
            PE += mass * GRAVITY * height;
        }

        PE
    }

    pub fn add_halfspace(&mut self, halfspace: &HalfSpace) {
        self.halfspaces = self.halfspaces.push(*halfspace);
    }

    /// Add contact point to rigid body.
    pub fn add_contact_point(&mut self, point: &ContactPoint) {
        for body in self.bodies.iter_mut() {
            if body.inertia.frame == point.frame {
                body.add_contact_point(point);
            }
        }
    }

    /// Add spring contact to rigid body.
    pub fn add_spring_contact(&mut self, spring_contact: &SpringContact) {
        for body in self.bodies.iter_mut() {
            if body.inertia.frame == spring_contact.frame {
                body.add_spring_contact(spring_contact);
            }
        }
    }

    /// Get the poses of each body
    pub fn poses(&self) -> Vec<Pose> {
        // TODO: use cached bodies_to_root
        let bodies_to_root = compute_bodies_to_root(&self);

        let mut poses: Vec<Pose> = vec![];
        for jointid in self.treejointids.iter() {
            let body_to_root = bodies_to_root.get(jointid).unwrap().mat;

            let rotation_matrix: Matrix3<Float> = body_to_root.fixed_view::<3, 3>(0, 0).into();
            let rotation = Rotation3::from_matrix(&rotation_matrix);
            let translation = body_to_root.fixed_view::<3, 1>(0, 3);

            let pose = Pose {
                rotation: UnitQuaternion::from_rotation_matrix(&rotation),
                translation: Vector3::from(translation),
            };
            poses.push(pose);
        }

        poses
    }
}

/// Computes the motion space of each joint, expressed in world frame.
pub fn compute_motion_subspaces(
    state: &MechanismState,
    bodies_to_root: &HashMap<usize, Transform3D>,
) -> HashMap<usize, GeometricJacobian> {
    let mut motion_subspaces = HashMap::new();
    for (bodyid, joint) in izip!(state.treejointids.iter(), state.treejoints.iter()) {
        let body_to_root = bodies_to_root.get(bodyid).unwrap();
        let ms_in_body = joint.motion_subspace();
        motion_subspaces.insert(*bodyid, ms_in_body.transform(body_to_root));
    }
    motion_subspaces
}

/// Compute the composite body inertia of each body, expressed in world frame
pub fn compute_crb_inertias(
    state: &MechanismState,
    inertias: &HashMap<usize, SpatialInertia>,
) -> HashMap<usize, SpatialInertia> {
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
///
/// Ref: Table 6.2 in Featherstone's Rigid Body Dynamics Algorithms, 2008
pub fn mass_matrix(
    state: &MechanismState,
    bodies_to_root: &HashMap<usize, Transform3D>,
) -> DMatrix<Float> {
    let n_v = state
        .v
        .iter()
        .map(|v| match v {
            JointVelocity::Float(_) => 1,
            JointVelocity::Spatial(_) => 6,
        })
        .sum();
    let mut mass_matrix = DMatrix::zeros(n_v, n_v);
    let motion_subspaces = compute_motion_subspaces(state, bodies_to_root);
    let inertias = compute_inertias(state, bodies_to_root);
    let crb_inertias = compute_crb_inertias(state, &inertias);

    let mut i_index = 0; // row index for putting in Hij
    let mut j_index = 0; // column index for putting in Hij
    for i in state.treejointids.iter() {
        let Ici = crb_inertias.get(i).unwrap();
        let Si = motion_subspaces.get(i).unwrap();
        let Fi = MomentumMatrix::mul(Ici, Si);
        for j in 1..=*i {
            let Sj = motion_subspaces.get(&j).unwrap();
            // let i_index: usize = (i - 1).try_into().unwrap();
            // let j_index: usize = (j - 1).try_into().unwrap();

            let Hij = Fi.transpose_mul(Sj);
            let nrows = Hij.nrows();
            let ncols = Hij.ncols();
            // println!("{}, {}, {}", i, j, Hij);
            mass_matrix
                .view_mut((i_index, j_index), (nrows, ncols))
                .copy_from(&Hij);

            // Incrementally fill out the mass matrix
            // TODO: Update when it is not just serial chain robot
            if i_index == j_index {
                i_index += nrows;
                j_index = 0;
            } else {
                j_index += ncols;
            }
        }
    }

    mass_matrix
}

#[cfg(test)]
mod mechanism_tests {

    use na::{dvector, vector, Matrix3};

    use crate::joint::{floating::FloatingJoint, revolute::RevoluteJoint};

    use super::*;

    #[test]
    fn mass_matrix_test() {
        // Arrange
        let m_body = 2.0; // mass
        let w_body = 1.0; // width
        let h_body = 0.1; // height

        let moment_x = (w_body * w_body + h_body * h_body) * m_body / 12.0;
        let moment_y = (w_body * w_body + h_body * h_body) * m_body / 12.0;
        let moment_z = (w_body * w_body + w_body * w_body) * m_body / 12.0;
        let moment_body = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
        let cross_part_body = vector![0.0, 0.0, 0.0];

        let body_frame = "body";
        let world_frame = "world";
        let body_to_world = Transform3D::identity(&body_frame, &world_frame);
        let body = RigidBody::new(SpatialInertia {
            frame: body_frame.to_string(),
            moment: moment_body,
            cross_part: cross_part_body,
            mass: m_body,
        });

        let m_leg = 1.0;
        let w_leg = 0.1;
        let h_leg = 1.0;

        let moment_x =
            m_leg * ((w_leg * w_leg + h_leg * h_leg) / 12.0 + (h_leg / 2.0 * h_leg / 2.0));
        let moment_y =
            m_leg * ((w_leg * w_leg + h_leg * h_leg) / 12.0 + (h_leg / 2.0 * h_leg / 2.0));
        let moment_z = (w_leg * w_leg + w_leg * w_leg) * m_leg / 12.0;
        let moment_leg = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
        let cross_part_leg = vector![0.0, 0.0, -h_leg / 2.0 * m_leg];
        let axis_leg = vector![0.0, 1.0, 0.0];

        let leg_frame = "leg";
        let leg_to_body = Transform3D::identity(&leg_frame, &body_frame);
        let leg = RigidBody::new(SpatialInertia {
            frame: leg_frame.to_string(),
            moment: moment_leg,
            cross_part: cross_part_leg,
            mass: m_leg,
        });

        let leg2_frame = "leg2";
        let leg2_to_leg = Transform3D {
            from: leg2_frame.to_string(),
            to: leg_frame.to_string(),
            mat: Transform3D::move_z(-h_leg),
        };
        let leg2 = RigidBody::new(SpatialInertia {
            frame: leg2_frame.to_string(),
            moment: moment_leg,
            cross_part: cross_part_leg,
            mass: m_leg,
        });

        let treejoints = dvector![
            Joint::FloatingJoint(FloatingJoint {
                init_mat: body_to_world.mat.clone(),
                transform: body_to_world,
            }),
            Joint::RevoluteJoint(RevoluteJoint {
                init_mat: leg_to_body.mat.clone(),
                transform: leg_to_body,
                axis: axis_leg,
            }),
            Joint::RevoluteJoint(RevoluteJoint {
                init_mat: leg2_to_leg.mat.clone(),
                transform: leg2_to_leg,
                axis: axis_leg,
            }),
        ];
        let bodies = dvector![body, leg, leg2];
        let halfspaces = dvector![];
        let state = MechanismState::new(treejoints, bodies, halfspaces);

        // Act
        let bodies_to_root = &compute_bodies_to_root(&state);
        let mass_matrix = mass_matrix(&state, bodies_to_root);

        // Assert
        assert_eq!(mass_matrix.nrows(), mass_matrix.ncols());
        assert_eq!(mass_matrix.nrows(), 6 + 1 + 1);
        assert_ne!(mass_matrix.fixed_view::<1, 6>(6, 0), DVector::zeros(6));
        assert_ne!(mass_matrix.fixed_view::<1, 1>(6, 6), DVector::zeros(1));
        assert_ne!(mass_matrix.fixed_view::<1, 6>(7, 0), DVector::zeros(6));
        assert_ne!(mass_matrix.fixed_view::<1, 1>(7, 6), DVector::zeros(1));
        assert_ne!(mass_matrix.fixed_view::<1, 1>(7, 7), DVector::zeros(1));
    }

    /// Ensure that poses() fn works correctly for a floating box
    #[test]
    #[ignore] // TODO: Add this test
    fn box_pose() {
        // Arrange

        // Act

        // Assert
    }

    /// Ensure that poses() fn works correctly for a double pendulum
    #[test]
    #[ignore] // TODO: Add this test
    fn double_pendulum_poses() {
        // Arrange

        // Act

        // Assert
    }
}
