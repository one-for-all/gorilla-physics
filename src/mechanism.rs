use std::collections::HashMap;
use std::collections::HashSet;

use crate::contact::ContactPoint;
use crate::contact::HalfSpace;
use crate::contact::SpringContact;
use crate::energy::spring_elastic_energy;
use crate::inertia::compute_inertias;
use crate::inertia::kinetic_energy;
use crate::inertia::SpatialInertia;
use crate::joint::cylindrical_constraint::Constraint;
use crate::joint::Joint;
use crate::joint::JointPosition;
use crate::joint::JointVelocity;
use crate::momentum::MomentumMatrix;
use crate::rigid_body::CollisionGeometry;
use crate::rigid_body::RigidBody;
use crate::spatial::geometric_jacobian::GeometricJacobian;
use crate::spatial::pose::Pose;
use crate::spatial::spatial_vector::SpatialVector;
use crate::spatial::transform::Transform3D;
use crate::spatial::twist::compute_joint_twists;
use crate::spatial::twist::compute_twists_wrt_world;
use crate::spatial::twist::Twist;
use crate::types::Float;
use crate::GRAVITY;
use crate::WORLD_FRAME;
use itertools::izip;
use na::DMatrix;

/// MechanismState stores the state information about the mechanism
/// Joint i's child body is body i.
/// Body 0 is by default world/root.
/// Does not support closed kinematic chain yet.
pub struct MechanismState {
    pub treejoints: Vec<Joint>,   // treejoints[i-1] = joint of joint number i
    pub treejointids: Vec<usize>, // consecutive numbers from 1 to len(treejoints)
    pub bodies: Vec<RigidBody>,   // bodies[i-1] = body of body number i
    pub parents: Vec<usize>, // parents[i-1] -> the parent body number for joint of joint number i
    pub supports: Vec<HashSet<usize>>, // supports[i-1] -> all the bodies that joint i supports
    pub q: Vec<JointPosition>, // joint configuration/position vector
    pub v: Vec<JointVelocity>, // joint velocity vector. Note: velocities are expressed in each body frame
    pub halfspaces: Vec<HalfSpace>,

    pub constraints: Vec<Constraint>,

    _bodies_to_root: Vec<Transform3D>,
    _bodies_to_root_obsolete: bool,

    _body_twists: Vec<Twist>,
    _body_twists_obsolete: bool,
}

impl MechanismState {
    /// Create a new MechanismState with zero initial condition
    pub fn new(treejoints: Vec<Joint>, bodies: Vec<RigidBody>) -> Self {
        let njoints = treejoints.len();
        let mut q = vec![];
        let mut v = vec![];
        let mut parents = vec![];
        let mut supports = vec![];
        for (index, j) in treejoints.iter().enumerate() {
            let jointid = index + 1;
            let bodyid = jointid;
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
                Joint::FixedJoint(_) => {
                    q.push(JointPosition::None);
                    v.push(JointVelocity::None);
                }
            }

            // Check that joint i has child body i
            assert!(
                j.transform().from == bodies[index].inertia.frame,
                "joint {}'s child body is not body {}\n",
                index + 1,
                index + 1
            );

            // Find the parent of each joint
            if j.transform().to == WORLD_FRAME {
                parents.push(0);
            } else {
                let mut has_parent = false;
                for (body_index, body) in bodies.iter().enumerate() {
                    if j.transform().to == body.inertia.frame {
                        parents.push(body_index + 1);
                        has_parent = true;
                        break;
                    }
                }
                assert!(
                    has_parent,
                    "joint {} has no parent body of frame {}\n",
                    index + 1,
                    j.transform().to
                );
            }

            // Update supports array
            let mut cur_bodyid = bodyid;
            supports.push(HashSet::from([bodyid]));
            while parents[cur_bodyid - 1] != 0 {
                let parentbodyid = parents[cur_bodyid - 1];
                supports[parentbodyid - 1].insert(bodyid);
                cur_bodyid = parentbodyid;
            }
        }

        let mut state = MechanismState {
            treejoints,
            treejointids: Vec::from_iter(1..njoints + 1),
            bodies,
            parents,
            supports,
            q,
            v,
            halfspaces: vec![],
            constraints: vec![],
            _bodies_to_root: vec![],
            _bodies_to_root_obsolete: true,
            _body_twists: vec![],
            _body_twists_obsolete: true,
        };

        let bodies_to_root = state.get_bodies_to_root();
        state.update_collider_poses(&bodies_to_root);
        state.update_visual_poses(&bodies_to_root);
        state
    }

    /// Compute the transforms from body frames to root, i.e. the isometries of the body frames
    /// Return the cached data, if not obsolete.
    pub fn get_bodies_to_root(&mut self) -> Vec<Transform3D> {
        if !self._bodies_to_root_obsolete {
            return self._bodies_to_root.clone(); // TODO(efficiency): remove the need for explicit cloning. Maybe return a reference?
        }

        let mut bodies_to_root = vec![Transform3D::identity("world", "world")];
        for (jointid, joint) in izip!(self.treejointids.iter(), self.treejoints.iter()) {
            let parentbodyid = self.parents[jointid - 1];
            let parent_to_root = &bodies_to_root[parentbodyid];
            let body_to_root = parent_to_root * &joint.transform();
            bodies_to_root.push(body_to_root);
        }

        self._bodies_to_root = bodies_to_root;
        self._bodies_to_root_obsolete = false;

        self._bodies_to_root.clone()
    }

    /// Compute the twist of each body with respect to the world frame,
    /// expressed in the world frame
    /// Note: the first twist is world
    pub fn get_body_twists(&mut self) -> Vec<Twist> {
        if !self._body_twists_obsolete {
            return self._body_twists.clone(); // TODO(efficiency): remove the need for explicit cloning. Maybe return a reference?
        }

        let bodies_to_root = self.get_bodies_to_root();
        let joint_twists = compute_joint_twists(self);
        let body_twists = compute_twists_wrt_world(self, &bodies_to_root, &joint_twists);

        self._body_twists = body_twists;
        self._body_twists_obsolete = false;

        self._body_twists.clone()
    }

    pub fn get_bodies_to_root_no_update(&self) -> Vec<Transform3D> {
        assert!(
            !self._bodies_to_root_obsolete,
            "trying to retrieve obsolete bodies_to_root data"
        );
        self._bodies_to_root.clone()
    }

    pub fn new_with_constraint(
        treejoints: Vec<Joint>,
        bodies: Vec<RigidBody>,
        constraints: Vec<Constraint>,
    ) -> Self {
        let mut state = Self::new(treejoints, bodies);
        state.constraints = constraints;
        state
    }

    pub fn update(&mut self, q: &Vec<JointPosition>, v: &Vec<JointVelocity>) {
        self._bodies_to_root_obsolete = true;
        self._body_twists_obsolete = true;
        self.update_q(q);
        self.v = v.clone();
    }

    pub fn update_q(&mut self, q: &Vec<JointPosition>) {
        self.q = q.clone();
        for (joint, qi) in izip!(self.treejoints.iter_mut(), q.iter()) {
            match joint {
                Joint::RevoluteJoint(j) => {
                    if let JointPosition::Float(qi) = qi {
                        j.update(*qi)
                    } else {
                        panic!("Revolute joint expects a Float position");
                    }
                }
                Joint::PrismaticJoint(j) => {
                    if let JointPosition::Float(qi) = qi {
                        j.update(*qi)
                    } else {
                        panic!("Prismatic joint expects a Float position");
                    }
                }
                Joint::FloatingJoint(j) => {
                    if let JointPosition::Pose(qi) = qi {
                        j.update(qi)
                    } else {
                        panic!("Floating joint expects a Pose position");
                    }
                }
                Joint::FixedJoint(_) => {
                    if !matches!(qi, JointPosition::None) {
                        panic!("Fixed joint expects a None position");
                    }
                }
            }
        }

        self._bodies_to_root_obsolete = true;
        self._body_twists_obsolete = true;

        let bodies_to_root = self.get_bodies_to_root();
        self.update_collider_poses(&bodies_to_root);
        self.update_visual_poses(&bodies_to_root);
    }

    /// Updates the collider of each body to have the body's pose
    pub fn update_collider_poses(&mut self, bodies_to_root: &Vec<Transform3D>) {
        for (jointid, body) in izip!(self.treejointids.iter(), self.bodies.iter_mut()) {
            if let Some(collider) = body.collider.as_mut() {
                let body_to_root = &bodies_to_root[*jointid];
                let isometry = body_to_root.iso;
                match &mut collider.geometry {
                    CollisionGeometry::Cuboid(cuboid) => {
                        cuboid.isometry = isometry;
                    }
                    CollisionGeometry::Mesh(mesh) => {
                        mesh.update_isometry(&isometry);
                    }
                    CollisionGeometry::Sphere(sphere) => {
                        sphere.isometry = isometry;
                    }
                }
            }
        }
    }

    /// Updates the visual of each body to have the body's pose
    pub fn update_visual_poses(&mut self, bodies_to_root: &Vec<Transform3D>) {
        for (jointid, body) in izip!(self.treejointids.iter(), self.bodies.iter_mut()) {
            let body_to_root = &bodies_to_root[*jointid];
            let isometry = body_to_root.iso;
            // TODO: update only once as a whole, since they all share the same
            // isometry. Or just keep the isometry on the body instead.
            for visual_mesh in body.visual.iter_mut() {
                visual_mesh.update_isometry(&isometry);
            }
        }
    }

    pub fn set_joint_q(&mut self, jointid: usize, q: JointPosition) {
        match &mut self.treejoints[jointid - 1] {
            Joint::RevoluteJoint(j) => {
                if let JointPosition::Float(q) = q {
                    (*j).update(q)
                } else {
                    panic!("Revolute joint expects a Float position");
                }
            }
            Joint::PrismaticJoint(j) => {
                if let JointPosition::Float(q) = q {
                    (*j).update(q)
                } else {
                    panic!("Prismatic joint expects a Float position");
                }
            }
            Joint::FloatingJoint(j) => {
                if let JointPosition::Pose(q) = q {
                    (*j).update(&q)
                } else {
                    panic!("Floating joint expects a Pose position");
                }
            }
            Joint::FixedJoint(_) => {
                panic!("Should not try to set a fixed joint's position");
            }
        }
        self.q[jointid - 1] = q;

        self._bodies_to_root_obsolete = true;
        self._body_twists_obsolete = true;

        let bodies_to_root = self.get_bodies_to_root();
        self.update_collider_poses(&bodies_to_root);
        self.update_visual_poses(&bodies_to_root);
    }

    pub fn set_joint_v(&mut self, jointid: usize, v: JointVelocity) {
        self._body_twists_obsolete = true;
        self.v[jointid - 1] = v;
    }

    /// Computes the total kinetic energy of the system
    pub fn kinetic_energy(&self) -> Float {
        let mut KE = 0.0;

        let bodies_to_root = self.get_bodies_to_root_no_update();
        let joint_twists = compute_joint_twists(self);
        let twists = compute_twists_wrt_world(self, &bodies_to_root, &joint_twists);
        for (jointid, body) in izip!(self.treejointids.iter(), self.bodies.iter()) {
            let bodyid = jointid;
            let twist = &twists[*bodyid];
            let body_to_root = &bodies_to_root[*bodyid];
            let spatial_inertia = body.inertia.transform(&body_to_root);

            let ke = kinetic_energy(&spatial_inertia, &twist);
            KE += ke;
        }
        KE
    }

    pub fn gravitational_energy(&self) -> Float {
        let mut PE = 0.0;
        let bodies_to_root = self.get_bodies_to_root_no_update();
        for jointid in self.treejointids.iter() {
            let height = bodies_to_root[*jointid].trans().z;
            let mass = self.bodies[jointid - 1].inertia.mass;
            PE += mass * GRAVITY * height;
        }

        PE
    }

    /// Computes the total energy of all the springs in the mechanism
    pub fn spring_energy(&self) -> Float {
        let mut E = 0.0;
        for (jointid, joint) in izip!(self.treejointids.iter(), self.treejoints.iter()) {
            if let Joint::PrismaticJoint(joint) = joint {
                if let Some(spring) = &joint.spring {
                    let q = &self.q[jointid - 1];
                    E += spring_elastic_energy(spring.l, q.float(), spring.k);
                }
            }
        }

        E
    }

    pub fn add_halfspace(&mut self, halfspace: HalfSpace) {
        self.halfspaces.push(halfspace);
    }

    /// Add contact point to rigid body.
    pub fn add_contact_point(&mut self, point: ContactPoint) {
        for body in self.bodies.iter_mut() {
            if body.inertia.frame == point.frame {
                body.add_contact_point(point);
                return;
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
        let bodies_to_root = self.get_bodies_to_root_no_update();

        let mut poses: Vec<Pose> = vec![];
        for jointid in self.treejointids.iter() {
            let body_to_root = bodies_to_root[*jointid].iso;

            let pose = Pose {
                rotation: body_to_root.rotation,
                translation: body_to_root.translation.vector,
            };
            poses.push(pose);
        }

        poses
    }

    /// True if the mechanism has spring contacts.
    /// It is needed to make sure there is no spring contact before using a multi-step
    /// integration scheme, since spring contacts have states not exhibitied in
    /// (q, v)
    /// TODO: make this check unnecessary?
    pub fn has_spring_contacts(&self) -> bool {
        for body in self.bodies.iter() {
            if !body.spring_contacts.is_empty() {
                return true;
            }
        }
        return false;
    }

    /// Updates all the collision-enabled meshes' vertex positions with regard
    /// to their body isometries.
    /// This is done once, to avoid unnecessary repeated updates for every
    /// collision detection event.
    pub fn update_collidable_mesh_vertex_positions(&mut self) {
        for body in self.bodies.iter_mut() {
            if let Some(collider) = &mut body.collider {
                if !collider.enabled {
                    continue;
                }
                match &mut collider.geometry {
                    CollisionGeometry::Mesh(mesh) => {
                        mesh.update_vertex_positions();
                    }
                    _ => {}
                }
            }
        }
    }

    /// Find all the bodyids in the linkage from root to the body with `frame`
    pub fn linked_bodyids(&self, frame: &str) -> HashSet<usize> {
        let mut linked_bodyids = HashSet::new();
        let mut currentid: usize = *izip!(self.treejointids.iter(), self.bodies.iter())
            .find_map(|x| {
                if x.1.inertia.frame == frame {
                    return Some(x.0);
                }
                None
            })
            .unwrap();
        while currentid != 0 {
            linked_bodyids.insert(currentid);
            currentid = self.parents[currentid - 1];
        }

        linked_bodyids
    }
}

/// Computes the motion space of each joint, expressed in world frame.
pub fn compute_motion_subspaces(
    state: &MechanismState,
    bodies_to_root: &Vec<Transform3D>,
) -> HashMap<usize, GeometricJacobian> {
    let mut motion_subspaces = HashMap::new();
    for (bodyid, joint) in izip!(state.treejointids.iter(), state.treejoints.iter()) {
        let body_to_root = &bodies_to_root[*bodyid];
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
        let mut crb_inertia: SpatialInertia = inertia.clone();
        for (joint_index, parentbodyid) in state.parents.iter().enumerate() {
            let jointid = joint_index + 1;
            let childbodyid = jointid;
            if bodyid == parentbodyid {
                crb_inertia += crb_inertias.get(&childbodyid).unwrap();
            }
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
pub fn mass_matrix(state: &MechanismState, bodies_to_root: &Vec<Transform3D>) -> DMatrix<Float> {
    let n_v = state
        .v
        .iter()
        .map(|v| match v {
            JointVelocity::Float(_) => 1,
            JointVelocity::Spatial(_) => 6,
            JointVelocity::None => 0,
        })
        .sum();
    let mut mass_matrix = DMatrix::zeros(n_v, n_v);
    let motion_subspaces = compute_motion_subspaces(state, bodies_to_root);
    let inertias = compute_inertias(state, bodies_to_root);
    let crb_inertias = compute_crb_inertias(state, &inertias);

    let mut i_index = 0; // row index for putting in Hij
    let mut j_index = 0; // column index for putting in Hij
    for i in state.treejointids.iter() {
        if matches!(state.treejoints.get(i - 1).unwrap(), Joint::FixedJoint(_)) {
            continue;
        }
        let Ici = crb_inertias.get(i).unwrap();
        let Si = motion_subspaces.get(i).unwrap();
        let Fi = MomentumMatrix::mul(Ici, Si);
        for j in 1..=*i {
            if matches!(state.treejoints.get(j - 1).unwrap(), Joint::FixedJoint(_)) {
                continue;
            }
            let Sj = motion_subspaces.get(&j).unwrap();
            let nrows = Si.dim();
            let ncols = Sj.dim();

            // if joint j supports body i
            if state.supports[j - 1].contains(i) {
                let Hij = Fi.transpose_mul(Sj);
                mass_matrix
                    .view_mut((i_index, j_index), (nrows, ncols))
                    .copy_from(&Hij);
            }

            // Incrementally fill out the mass matrix
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

    use na::{vector, DVector, Isometry3, Matrix3, Vector3};

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
        let axis_leg = Vector3::y_axis();
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
            iso: Isometry3::translation(0., 0., -h_leg),
        };
        let leg2 = RigidBody::new(SpatialInertia {
            frame: leg2_frame.to_string(),
            moment: moment_leg,
            cross_part: cross_part_leg,
            mass: m_leg,
        });

        let treejoints = vec![
            Joint::FloatingJoint(FloatingJoint {
                init_iso: body_to_world.iso,
                transform: body_to_world,
            }),
            Joint::RevoluteJoint(RevoluteJoint::new(leg_to_body, axis_leg)),
            Joint::RevoluteJoint(RevoluteJoint::new(leg2_to_leg, axis_leg)),
        ];
        let bodies = vec![body, leg, leg2];
        let state = MechanismState::new(treejoints, bodies);

        // Act
        let bodies_to_root = &state.get_bodies_to_root_no_update();
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

    /// Verify supports array is computed correctly fn on the following mechanism
    /// 3         5
    /// |         |
    /// 2 -- 1 -- 4
    #[test]
    fn test_supports() {
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
            RigidBody::new_sphere(1., 1., "5"),
        ];
        let state = MechanismState::new(treejoints, bodies);

        // Act
        let supports = state.supports;

        // Assert
        assert_eq!(
            supports,
            vec![
                HashSet::from([1, 2, 3, 4, 5]),
                HashSet::from([2, 3]),
                HashSet::from([3]),
                HashSet::from([4, 5]),
                HashSet::from([5])
            ]
        )
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
