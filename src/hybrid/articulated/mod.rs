use std::collections::HashMap;

use itertools::izip;
use na::{vector, DMatrix, DVector, Matrix6xX, UnitQuaternion, Vector3};

use crate::{
    hybrid::rigid::Rigid,
    inertia::SpatialInertia,
    joint::{
        constraint::{Constraint, RangeConstraint, RelativeRangeConstraint},
        Joint, JointPosition, JointVelocity,
    },
    spatial::{
        geometric_jacobian::{Momentum, MotionSubspace},
        pose::Pose,
        spatial_vector::SpatialVector,
    },
    types::Float,
    util::{
        inertia_mul, quaternion_derivative, spatial_force_cross, spatial_motion_cross,
        spatial_to_linear_velocity,
    },
    GRAVITY, WORLD_FRAME,
};

pub mod helper;

/// A hand laid on one body from outside the simulation: a damped spring pulling
/// a point on that body towards wherever the hand is asking for it, held until
/// it is replaced or cleared.
///
/// It is a spring the solve evaluates rather than a force handed over ready-made
/// because **both of its ends have to be the ones this step has**. The caller is
/// not part of the physics -- a cursor dragging a link -- and it runs at the
/// display's frame rate, which at `run(60, 0)` is sixty steps behind. A force
/// worked out up there is a sixty-step-old answer to where the body was and how
/// fast it was going, and on a light body free to turn that pumps energy in
/// rather than taking it out: a 20 g part grabbed a centimetre off its centre
/// spins up to a NaN inside a second. So the caller says where it is holding and
/// where it is pulling to, and the force is worked out here, every step.
///
/// A step does not spend it, so whoever set it is the one who takes it away.
pub struct AppliedSpring {
    pub body: usize,            // index into `Articulated::bodies`
    /// Where the hand holds the body, in that body's own frame -- so it stays
    /// the same material point as the body turns under it, without the caller
    /// having to say so again. Set from a world point by `set_applied_spring`.
    pub point: Vector3<Float>,
    pub target: Vector3<Float>, // where it is pulling `point` to, world frame
    pub stiffness: Float,       // N/m
    pub damping: Float,         // N s/m, against the velocity of `point`
    pub max_force: Float,       // N, the cap on the two of them together
}

/// bodies connected to each other through joints
pub struct Articulated {
    pub bodies: Vec<Rigid>, // all the rigid bodies in it
    pub joints: Vec<Joint>, // joints[i] connects bodies[i] to its parent

    pub parents: Vec<usize>, // parents[i] -> index of parent body of body i, and parents[i] == i means parent is world

    pub show_visual: bool,

    jacobians: Vec<Matrix6xX<Float>>, // Jacobian from each joint v to body spatial twist expressed in world frame
    pub mass_matrix: DMatrix<Float>,

    pub constraints: Vec<Constraint>, // Joint constraints for kinematic loops
    pub relative_range_constraints: Vec<RelativeRangeConstraint>, // range constraint between two q
    pub range_constraints: Vec<RangeConstraint>, // range constraint on a single q

    /// An outside hand on one of the bodies, if there is one. It travels with
    /// this articulated rather than being held against its index somewhere
    /// above, so re-ordering a `Hybrid`'s articulated cannot leave it pointing
    /// at the wrong robot.
    pub applied_spring: Option<AppliedSpring>,

    /// The bodies welded to the world where they stand, as indices into
    /// `bodies`. Held here for the same reason `applied_spring` is.
    ///
    /// A weld is a constraint rather than a swapped-in `FixedJoint`, which is
    /// what lets *any* body carry one rather than only a floating root, and what
    /// makes it reversible: `dof`, `offsets` and the mass matrix keep their
    /// shapes, so nothing keyed by joint index has to be rebuilt when a body is
    /// fixed or released. `Hybrid::step` is where it is enforced.
    pub fixed_to_world: Vec<usize>,
}

impl Articulated {
    pub fn new(bodies: Vec<Rigid>, joints: Vec<Joint>) -> Self {
        let mut frame_to_id = HashMap::new();
        for (i, body) in bodies.iter().enumerate() {
            frame_to_id.insert(body.inertia.frame.clone(), i);
        }
        let mut parents = vec![0; bodies.len()];
        for joint in joints.iter() {
            let parent = &joint.transform().to;
            let child = &joint.transform().from;
            if parent == WORLD_FRAME {
                parents[frame_to_id[child]] = frame_to_id[child];
            } else {
                parents[frame_to_id[child]] = frame_to_id[parent];
            }
        }

        let mut state = Self {
            bodies: bodies,
            joints: joints,
            parents: parents,
            show_visual: true,
            jacobians: vec![],
            mass_matrix: DMatrix::<Float>::zeros(0, 0),
            constraints: vec![],
            relative_range_constraints: vec![],
            range_constraints: vec![],
            applied_spring: None,
            fixed_to_world: vec![],
        };

        state.update_body_states();
        state
    }

    /// Take hold of one body from outside at world point `point`, replacing
    /// whatever was there. It stays until it is cleared -- see
    /// [`AppliedSpring`].
    ///
    /// `point` is read against the pose the body is in now and kept as the
    /// material point it lands on, so this is called once at the taking hold
    /// and not again; `aim_applied_spring` is what follows a cursor. A `body`
    /// past the end takes hold of nothing rather than panicking, since the
    /// caller is outside the simulation.
    pub fn set_applied_spring(
        &mut self,
        body: usize,
        point: Vector3<Float>,
        target: Vector3<Float>,
        stiffness: Float,
        damping: Float,
        max_force: Float,
    ) {
        let Some(rigid) = self.bodies.get(body) else {
            self.applied_spring = None;
            return;
        };
        let iso = rigid.pose.to_isometry();
        self.applied_spring = Some(AppliedSpring {
            body,
            point: iso.rotation.inverse() * (point - iso.translation.vector),
            target,
            stiffness,
            damping,
            max_force,
        });
    }

    /// Pull whatever is being held towards a different world point. Nothing
    /// held, nothing to aim.
    pub fn aim_applied_spring(&mut self, target: Vector3<Float>) {
        if let Some(applied) = self.applied_spring.as_mut() {
            applied.target = target;
        }
    }

    pub fn clear_applied_spring(&mut self) {
        self.applied_spring = None;
    }

    /// Weld body `body` to the world where it stands, and hold it there until it
    /// is released -- see [`Articulated::fixed_to_world`].
    ///
    /// The pose is not recorded anywhere, because it does not have to be: what
    /// the solve is given is that the body's velocity is zero, and a body that
    /// never moves is still in the pose it was fixed in. A `body` past the end,
    /// or one already fixed, welds nothing rather than panicking, since the
    /// caller is outside the simulation.
    pub fn fix_to_world(&mut self, body: usize) {
        if body < self.bodies.len() && !self.fixed_to_world.contains(&body) {
            self.fixed_to_world.push(body);
        }
    }

    /// Let body `body` go again. A step does not spend a weld, so this is the
    /// only thing that ends one.
    pub fn release_from_world(&mut self, body: usize) {
        self.fixed_to_world.retain(|&fixed| fixed != body);
    }

    pub fn is_fixed_to_world(&self, body: usize) -> bool {
        self.fixed_to_world.contains(&body)
    }

    pub fn add_constraints(&mut self, constraints: Vec<Constraint>) {
        self.constraints.extend(constraints);
    }

    pub fn add_relative_range_constraints(&mut self, constraints: Vec<RelativeRangeConstraint>) {
        self.relative_range_constraints.extend(constraints);
    }

    pub fn add_range_constraints(&mut self, constraints: Vec<RangeConstraint>) {
        self.range_constraints.extend(constraints);
    }

    fn update_body_states(&mut self) {
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
    }

    pub fn free_velocity(
        &self,
        dt: Float,
        tau: DVector<Float>,
        gravity_enabled: bool,
    ) -> DVector<Float> {
        // Compute dynamics bias c(q, v)
        // First, compute bias accelerations, i.e. acceleration of each body when no external force,  and no joint acceleration
        let mut bias_accels: Vec<SpatialVector> = vec![];
        for (i, joint) in self.joints.iter().enumerate() {
            let parent = self.parents[i];
            let pose = self.bodies[i].pose;
            let joint_twist = joint.twist().transform(&pose);
            let body_twist = self.bodies[i].twist;
            let coriolis_accel = spatial_motion_cross(&body_twist, &joint_twist);
            let bias_accel = if parent == i {
                // add inv gravity accel to base to simulate gravity force
                if gravity_enabled {
                    SpatialVector::linear(vector![0., 0., GRAVITY]) + coriolis_accel
                } else {
                    coriolis_accel
                }
            } else {
                bias_accels[parent] + coriolis_accel
            };
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

        // Solve for velocity
        let total_tau = tau + self.applied_generalized_force(dt) - c;
        let mass_matrix = self.mass_matrix.clone();
        let vdot = mass_matrix.cholesky().unwrap().solve(&total_tau);

        self.v() + vdot * dt
    }

    /// What `applied_spring` amounts to at the joints, zero if there is none.
    ///
    /// The force is the spring's, worked out here rather than passed in: the
    /// held point is carried into the world by the pose the body is in *this
    /// step*, the stretch is from there to where the hand is pulling, and the
    /// damping is against the velocity that point has -- which is the whole
    /// reason this lives in the solve, see [`AppliedSpring`]. The cap is on the
    /// two of them together, so the damping half can hold the body back as hard
    /// as the spring half can pull it.
    ///
    /// A force f acting at world point p is then the wrench (p x f, f) about the
    /// world origin -- the same construction contact.rs makes for a contact
    /// force -- and the body's own Jacobian carries it back to the joints as
    /// J^T w, which is where a generalized force belongs. Only the bodies
    /// between the world and the one pushed have a row that is not zero, since
    /// that is all `total_jacobian_to_body` fills in.
    ///
    /// The damping is also bounded here, and by the step: a damper of
    /// coefficient c takes c*v*dt of momentum out of a body that has m*v, so
    /// past c = m/dt it removes more than is there and the next step gets it
    /// back with the sign flipped -- which is not damping but a pump, and the
    /// lighter the body the sooner it starts. The quarter is margin for the
    /// hand being off the centre of mass, where the mass a push at a point sees
    /// is a fraction of the body's own.
    ///
    /// A `body` past the end is ignored rather than a panic: it can only come
    /// from outside the simulation, and dropping a force is a better answer
    /// there than taking the caller down with it.
    fn applied_generalized_force(&self, dt: Float) -> DVector<Float> {
        let Some(applied) = self.applied_spring.as_ref() else {
            return DVector::zeros(self.dof());
        };
        if applied.body >= self.bodies.len() {
            return DVector::zeros(self.dof());
        }

        let rigid = &self.bodies[applied.body];
        let iso = rigid.pose.to_isometry();
        let p = iso.rotation * applied.point + iso.translation.vector;
        let v = spatial_to_linear_velocity(&rigid.twist, &p);
        let damping = applied.damping.min(rigid.inertia.mass / (4. * dt));
        let mut f = (applied.target - p) * applied.stiffness - v * damping;
        let magnitude = f.norm();
        if magnitude > applied.max_force {
            f *= applied.max_force / magnitude;
        }

        let torque = p.cross(&f);
        let wrench = vector![torque[0], torque[1], torque[2], f[0], f[1], f[2]];
        self.total_jacobian_to_body(applied.body).transpose() * wrench
    }

    pub fn update_mass_matrix(&mut self) {
        // Compute and store motion subspaces, expressed in world frame
        let motion_subspaces: Vec<MotionSubspace> = izip!(self.joints.iter(), self.bodies.iter())
            .map(|(j, b)| j.motion_subspace().as_s().transform(&b.pose))
            .collect();

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
        for dof in dofs.iter() {
            indices.push(i);
            i += dof;
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

            // A drivetrain's reflected rotor inertia accelerates with its own
            // joint alone, so it couples to nothing else and only adds to that
            // joint's diagonal. Armature is single-dof, hence the lone entry.
            // A fixed joint has no row at all, and `indices` puts every one of
            // them that follows the last moving joint at `dof` -- one past the
            // end of the matrix -- so it has to be skipped rather than added 0.
            if idof > 0 {
                mass_matrix[(iindex, iindex)] += self.joints[i].armature();
            }

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

        self.mass_matrix = mass_matrix;
    }

    /// Compute the body twists in world frame, at the given joint velocities
    pub fn body_twists_at(&self, v: &DVector<Float>) -> Vec<SpatialVector> {
        let mut iv = 0;
        let mut body_twists = vec![];
        for (i, joint) in self.joints.iter().enumerate() {
            let parent = self.parents[i];
            let pose = self.bodies[i].pose;
            let mut joint = (*joint).clone();
            match &mut joint {
                Joint::FixedJoint(_) => {}
                Joint::RevoluteJoint(j) => j.v = v[iv],
                Joint::PrismaticJoint(j) => j.v = v[iv],
                Joint::FloatingJoint(j) => {
                    j.v = SpatialVector::new(
                        vector![v[iv], v[iv + 1], v[iv + 2]],
                        vector![v[iv + 3], v[iv + 4], v[iv + 5]],
                    )
                }
            }
            let joint_twist = joint.twist().transform(&pose);
            let body_twist = if parent == i {
                joint_twist
            } else {
                body_twists[parent] + joint_twist
            };
            body_twists.push(body_twist);
            iv += joint.dof();
        }
        body_twists
    }

    /// Compute the current body twists in world frame
    pub fn body_twists(&self) -> Vec<SpatialVector> {
        self.bodies.iter().map(|b| b.twist).collect()
    }

    /// Total degrees of freedom
    pub fn dof(&self) -> usize {
        self.joints.iter().map(|j| j.dof()).sum()
    }

    /// Offset index of each joint
    pub fn offsets(&self) -> Vec<usize> {
        let mut offset = 0;
        let mut offsets = vec![];
        for joint in self.joints.iter() {
            offsets.push(offset);
            offset += joint.dof();
        }
        offsets
    }

    /// Velocity vector of the system
    pub fn v(&self) -> DVector<Float> {
        let mut v = vec![];
        for joint in self.joints.iter() {
            v.extend(joint.v().iter());
        }
        DVector::from_vec(v)
    }

    /// position vector of the system
    pub fn q(&self) -> DVector<Float> {
        let mut q = vec![];
        for joint in self.joints.iter() {
            q.extend(joint.q().iter());
        }
        DVector::from_vec(q)
    }

    /// i-th joint's q vector
    pub fn joint_q(&self, i: usize) -> DVector<Float> {
        self.joints[i].q()
    }

    pub fn mass(&self) -> Float {
        self.bodies.iter().map(|b| b.inertia.mass).sum()
    }

    /// Update joint velocities, and integrate joint positions through dt
    pub fn integrate(&mut self, v: DVector<Float>, dt: Float) {
        let mut i = 0;
        for joint in self.joints.iter_mut() {
            match joint {
                Joint::FixedJoint(_) => {}
                Joint::RevoluteJoint(j) => {
                    j.v = v[i];
                    j.set_q(j.get_q() + j.v * dt);
                }
                Joint::PrismaticJoint(j) => {
                    j.v = v[i];
                    j.set_q(j.get_q() + j.v * dt);
                }
                Joint::FloatingJoint(j) => {
                    j.v = SpatialVector::new(
                        vector![v[i], v[i + 1], v[i + 2]],
                        vector![v[i + 3], v[i + 4], v[i + 5]],
                    );
                    let quaternion_dot = quaternion_derivative(&j.get_q().rotation, &j.v.angular);
                    let translation_dot = j.get_q().rotation * j.v.linear;
                    let new_q = Pose {
                        translation: j.get_q().translation + translation_dot * dt,
                        rotation: UnitQuaternion::from_quaternion(
                            j.get_q().rotation.quaternion() + quaternion_dot * dt,
                        ),
                    };
                    j.set_q(new_q);
                }
            }

            let dof = joint.dof();
            i += dof;
        }

        self.update_body_states();
    }

    /// Jacobian from each joint v to body spatial twist expressed in world frame
    pub fn update_jacobians(&mut self) {
        self.jacobians = izip!(self.joints.iter(), self.bodies.iter())
            .map(|(j, b)| j.motion_subspace().as_s().transform(&b.pose).as_matrix())
            .collect();
    }

    /// Jacobian from total v to the total spatial twist of ith body, expressed in world frame
    pub fn total_jacobian_to_body(&self, i: usize) -> Matrix6xX<Float> {
        let dof = self.dof();
        let offsets = self.offsets();
        let jacobians = &self.jacobians;

        let mut H = Matrix6xX::zeros(dof);
        H.view_mut((0, offsets[i]), (6, self.joints[i].dof()))
            .copy_from(&jacobians[i]);
        let mut parent = i;
        while self.parents[parent] != parent {
            parent = self.parents[parent];
            H.view_mut((0, offsets[parent]), (6, self.joints[parent].dof()))
                .copy_from(&jacobians[parent]);
        }
        H
    }

    pub fn step(&mut self, dt: Float) {
        self.update_jacobians();
        self.update_mass_matrix();

        let gravity_enabled = true;
        let v = self.free_velocity(dt, DVector::zeros(self.dof()), gravity_enabled);
        self.integrate(v, dt);
    }

    pub fn set_joint_v(&mut self, i: usize, v: JointVelocity) {
        match &mut self.joints[i] {
            Joint::FixedJoint(_) => panic!("can't set fixed joint v"),
            Joint::RevoluteJoint(j) => j.v = v.float(),
            Joint::PrismaticJoint(j) => j.v = v.float(),
            Joint::FloatingJoint(j) => j.v = *v.spatial(),
        }
        self.update_body_states();
    }

    pub fn set_joint_q(&mut self, i: usize, q: JointPosition) {
        match &mut self.joints[i] {
            Joint::FixedJoint(_) => panic!("can't set fixed joint v"),
            Joint::RevoluteJoint(j) => j.set_q(q.float()),
            Joint::PrismaticJoint(j) => j.set_q(q.float()),
            Joint::FloatingJoint(j) => j.set_q(*q.pose()),
        }
        self.update_body_states();
    }

    /// Reset all joints to default state
    pub fn reset(&mut self) {
        for joint in self.joints.iter_mut() {
            joint.reset();
        }
    }

    pub fn potential_energy(&self) -> Float {
        self.bodies.iter().map(|b| b.potential_energy()).sum()
    }

    pub fn kinetic_energy(&self) -> Float {
        self.bodies.iter().map(|b| b.kinetic_energy()).sum()
    }

    pub fn total_energy(&self) -> Float {
        self.potential_energy() + self.kinetic_energy()
    }
}

#[cfg(test)]
mod articulated_tests {
    use na::{vector, Vector3};

    use crate::{
        assert_close, assert_vec_close,
        hybrid::{articulated::Articulated, rigid::Rigid},
        joint::{Joint, JointVelocity},
        spatial::transform::Transform3D,
        types::Float,
        GRAVITY, PI, WORLD_FRAME,
    };

    #[test]
    fn cart() {
        // Arrange
        let bodies = vec![Rigid::new_cuboid_at(
            &vector![0., 0., 0.],
            1.,
            1.,
            0.1,
            0.1,
            "cart",
        )];
        let cart_to_world = Transform3D::identity("cart", WORLD_FRAME);
        let joints = vec![Joint::new_prismatic(cart_to_world, Vector3::x_axis())];
        let mut state = Articulated::new(bodies, joints);
        let v = 1.0;
        state.set_joint_v(0, JointVelocity::Float(v));

        // Act
        let final_time = 2.0;
        let dt = 1e-3;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            state.step(dt);
        }

        // Assert
        assert_vec_close!(state.v(), vec![v], 1e-3);
        assert_vec_close!(state.q(), vec![final_time * v], 1e-3);
    }

    #[test]
    fn pendulum() {
        // Arrange
        let l = 1.0;
        let bodies = vec![Rigid::new_sphere_at(
            &vector![l, 0., 0.],
            1.,
            1.,
            "pendulum",
        )];
        let sphere_to_world = Transform3D::identity("pendulum", WORLD_FRAME);
        let joints = vec![Joint::new_revolute(sphere_to_world, Vector3::y_axis())];
        let mut state = Articulated::new(bodies, joints);
        let total_energy = state.total_energy();

        // Act
        let mut max_angle = 0.;
        let final_time = 2.0;
        let dt = 1e-3;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            state.step(dt);
            let angle = state.q()[0];
            if angle > max_angle {
                max_angle = angle;
            }
        }

        // Assert
        assert_close!(max_angle, PI, 1e-3);
        assert_close!(total_energy, state.total_energy(), 1e-2);
    }

    #[test]
    fn double_pendulum() {
        // Arrange
        let l = 1.0;
        let r = 0.1;
        let bodies = vec![
            Rigid::new_sphere_at(&vector![l, 0., 0.], 1.0, r, "pendulum"),
            Rigid::new_sphere_at(&vector![l, 0., 0.], 1.0, r, "pendulum2"),
        ];
        let pendulum_to_world = Transform3D::identity("pendulum", WORLD_FRAME);
        let pendulum2_to_pendulum = Transform3D::move_x("pendulum2", "pendulum", l);
        let joints = vec![
            Joint::new_revolute(pendulum_to_world, Vector3::y_axis()),
            Joint::new_revolute(pendulum2_to_pendulum, Vector3::y_axis()),
        ];
        let mut state = Articulated::new(bodies, joints);
        let total_energy = state.total_energy();

        // Act
        let final_time = 2.0;
        let dt = 1e-3;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            state.step(dt);
        }

        // Assert
        assert_close!(total_energy, state.total_energy(), 1e-1);
    }

    /// A spring on the centre of mass, along the one axis a cart can move on,
    /// takes it to where it was asked for and leaves it there.
    #[test]
    fn an_applied_spring_pulls_a_body_to_where_it_is_asked_for() {
        // Arrange
        let m = 1.0;
        let bodies = vec![Rigid::new_cuboid_at(
            &vector![0., 0., 0.],
            m,
            1.,
            0.1,
            0.1,
            "cart",
        )];
        let cart_to_world = Transform3D::identity("cart", WORLD_FRAME);
        let joints = vec![Joint::new_prismatic(cart_to_world, Vector3::x_axis())];
        let mut state = Articulated::new(bodies, joints);

        // Act
        let x = 0.05;
        state.set_applied_spring(0, vector![0., 0., 0.], vector![x, 0., 0.], 200., 8., 20.);
        let final_time = 5.0;
        let dt = 1e-3;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            state.step(dt);
        }

        // Assert
        assert_vec_close!(state.q(), vec![x], 1e-3);
        assert_vec_close!(state.v(), vec![0.], 1e-3);
    }

    /// The lever arm is the half that is easy to get wrong, and the cheapest
    /// way to pin it is to cancel gravity with it: a force of m*g straight up
    /// at the bob is the same wrench gravity puts there and the opposite sign,
    /// so a pendulum that would otherwise swing to PI does not move at all.
    /// Get `point x force` backwards and it swings twice as fast instead.
    ///
    /// A spring of stiffness k asked for the bob m*g/k above where it already
    /// is pulls with exactly that force for as long as the bob does not move --
    /// which, if the cancellation holds, it does not.
    #[test]
    fn an_applied_spring_at_a_point_cancels_gravity_there() {
        // Arrange
        let l = 1.0;
        let m = 1.0;
        let bodies = vec![Rigid::new_sphere_at(&vector![l, 0., 0.], m, 1., "pendulum")];
        let sphere_to_world = Transform3D::identity("pendulum", WORLD_FRAME);
        let joints = vec![Joint::new_revolute(sphere_to_world, Vector3::y_axis())];
        let mut state = Articulated::new(bodies, joints);

        // Act
        let k = 100.;
        state.set_applied_spring(
            0,
            vector![l, 0., 0.],
            vector![l, 0., m * GRAVITY / k],
            k,
            0.,
            100.,
        );
        let final_time = 2.0;
        let dt = 1e-3;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            state.step(dt);
        }

        // Assert
        assert_vec_close!(state.q(), vec![0.], 1e-6);
        assert_vec_close!(state.v(), vec![0.], 1e-6);

        // And it is the spring doing that, not the joint standing still on its
        // own: taken away, the pendulum swings as `pendulum` above says it does.
        state.clear_applied_spring();
        let mut max_angle = 0.;
        for _s in 0..num_steps {
            state.step(dt);
            if state.q()[0] > max_angle {
                max_angle = state.q()[0];
            }
        }
        assert_close!(max_angle, PI, 1e-3);
    }

    /// A 20 g printed part held by one corner -- a dropped object being nudged
    /// -- settles where the hand asks for it instead of spinning up.
    ///
    /// The arrangement is the browser's: the hand is taken hold of and aimed
    /// from up in the page, and the solve steps sixty times per animation frame
    /// underneath it. That is why neither end of the spring can be the caller's
    /// to work out. Against a sixty-step-old point and velocity it pumps energy
    /// in rather than taking it out, and the part reaches a NaN mass matrix
    /// within a second of being touched (roadmap#10).
    #[test]
    fn a_light_body_held_by_a_corner_does_not_spin_up() {
        // Arrange
        let m = 0.02;
        let w = 0.05;
        let corner = vector![w / 2., w / 2., 0.];
        let mut state = Articulated::new_cube_at("block", m, w, &vector![0., 0., 0.5]);
        let held = |state: &Articulated| {
            let iso = state.bodies[0].pose.to_isometry();
            iso.rotation * corner + iso.translation.vector
        };
        let target = held(&state) + vector![0.05, 0., 0.];

        // Act -- the gains and the cap are the page's own, from www/src/nudge.ts
        let dt = 1. / 3600.;
        state.set_applied_spring(0, held(&state), target, 200., 8., 20.);
        let mut fastest: Float = 0.;
        for _frame in 0..120 {
            for _substep in 0..60 {
                state.step(dt);
            }
            fastest = fastest.max(state.bodies[0].twist.angular.norm());
        }

        // Assert - the hand has the corner where it asked for it, to a millimetre
        assert_vec_close!(held(&state), target, 5e-3);
        // Assert - and the part is turning no faster than the swing of a thing
        // hanging by a corner, rather than winding up without limit. It does
        // swing: a rotation about the point being held moves that point not at
        // all, so there is nothing for the damping to take the energy out
        // through, exactly as with a real hand pinching one corner.
        assert!(fastest < 40., "wound up to {} rad/s", fastest);
    }
}
