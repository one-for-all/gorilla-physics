use std::collections::HashMap;

use itertools::izip;
use na::Vector3;
use na::{vector, DMatrix, DVector, UnitQuaternion};

use crate::{
    hybrid::rigid::Rigid,
    inertia::SpatialInertia,
    joint::{Joint, JointVelocity},
    spatial::{
        geometric_jacobian::{Momentum, MotionSubspace},
        pose::Pose,
        spatial_vector::SpatialVector,
    },
    types::Float,
    util::{inertia_mul, quaternion_derivative, spatial_force_cross, spatial_motion_cross},
    GRAVITY, WORLD_FRAME,
};

/// bodies connected to each other through joints
pub struct Articulated {
    pub bodies: Vec<Rigid>, // all the rigid bodies in it
    pub joints: Vec<Joint>, // joints[i] connects bodies[i] to its parent

    pub parents: Vec<usize>, // parents[i] -> index of parent body of body i, and parents[i] == i means parent is world
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

        Self {
            bodies: bodies,
            joints: joints,
            parents: parents,
        }
    }

    pub fn free_velocity(&mut self, dt: Float) -> DVector<Float> {
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

        let mass_matrix = self.mass_matrix();

        let vdot = mass_matrix.cholesky().unwrap().solve(&-c);

        self.v() + vdot * dt
    }

    pub fn mass_matrix(&self) -> DMatrix<Float> {
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
        mass_matrix
    }

    /// Total degrees of freedom
    pub fn dof(&self) -> usize {
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

    /// position vector of the system
    pub fn q(&self) -> DVector<Float> {
        let mut q = vec![];
        for joint in self.joints.iter() {
            q.extend(joint.q().iter());
        }
        DVector::from_vec(q)
    }

    /// Update joint velocities, and integrate joint positions through dt
    pub fn integrate(&mut self, v: DVector<Float>, dt: Float) {
        let mut i = 0;
        for joint in self.joints.iter_mut() {
            match joint {
                Joint::FixedJoint(_) => {}
                Joint::RevoluteJoint(j) => {
                    j.v = v[i];
                    j.q += j.v * dt;
                    j.update(j.q);
                }
                Joint::PrismaticJoint(j) => {
                    j.v = v[i];
                    j.q += j.v * dt;
                    j.update(j.q);
                }
                Joint::FloatingJoint(j) => {
                    j.v = SpatialVector::new(
                        vector![v[i], v[i + 1], v[i + 2]],
                        vector![v[i + 3], v[i + 4], v[i + 5]],
                    );
                    let quaternion_dot = quaternion_derivative(&j.q.rotation, &j.v.angular);
                    let translation_dot = j.q.rotation * j.v.linear;
                    let new_q = Pose {
                        translation: j.q.translation + translation_dot * dt,
                        rotation: UnitQuaternion::from_quaternion(
                            j.q.rotation.quaternion() + quaternion_dot * dt,
                        ),
                    };
                    j.q = new_q;
                    j.update(&new_q);
                }
            }

            let dof = joint.dof();
            i += dof;
        }

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

    pub fn step(&mut self, dt: Float) {
        let v = self.free_velocity(dt);
        self.integrate(v, dt);
    }

    pub fn set_joint_v(&mut self, i: usize, v: JointVelocity) {
        match &mut self.joints[i] {
            Joint::FixedJoint(_) => panic!("can't set fixed joint v"),
            Joint::RevoluteJoint(j) => j.v = v.float(),
            Joint::PrismaticJoint(j) => j.v = v.float(),
            Joint::FloatingJoint(j) => j.v = *v.spatial(),
        }
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
        PI, WORLD_FRAME,
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
    }
}
