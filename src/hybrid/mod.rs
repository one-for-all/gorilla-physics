use clarabel::{
    algebra::{CscMatrix, MatrixMathMut},
    solver::{
        DefaultSettingsBuilder, DefaultSolver, IPSolver,
        SupportedConeT::{self, SecondOrderConeT},
    },
};
use itertools::izip;
use na::{
    vector, DMatrix, DVector, Matrix1xX, Matrix3, Matrix3x6, Matrix3xX, Matrix6xX, UnitQuaternion,
    UnitVector3, Vector3,
};

use crate::{
    collision::halfspace::HalfSpace,
    flog,
    hybrid::{
        articulated::Articulated,
        cloth::Cloth,
        control::{ArticulatedController, NullArticulatedController},
        deformable::deformable_deformable_ccd,
        rigid::{rigid_cloth_ccd, rigid_deformable_cd},
    },
    spatial::{
        pose::Pose, spatial_vector::SpatialVector, twist::compute_twist_transformation_matrix,
    },
    types::Float,
    util::{quaternion_derivative, skew_symmetric, tangentials},
};

pub use deformable::Deformable;
pub use rigid::Rigid;

pub mod articulated;
pub mod builders;
pub mod cloth;
pub mod control;
pub mod deformable;
pub mod rigid;
pub mod visual;

pub struct Hybrid {
    pub rigid_bodies: Vec<Rigid>,
    pub articulated: Vec<Articulated>,
    pub deformables: Vec<Deformable>,
    pub cloths: Vec<Cloth>,

    pub halfspaces: Vec<HalfSpace>,

    pub controllers: Vec<Box<dyn ArticulatedController>>,

    gravity_enabled: bool,
}

impl Hybrid {
    /// A canonical hybrid system with 1 sphere rigid-body and 1 tetrahedron deformable
    pub fn new_canonical() -> Self {
        let rigid = Rigid::new_sphere(1., 1., "sphere");
        let deformable = Deformable::new_tetrahedron();

        Hybrid {
            rigid_bodies: vec![rigid],
            articulated: vec![],
            deformables: vec![deformable],
            cloths: vec![],
            halfspaces: vec![],
            controllers: vec![],
            gravity_enabled: true,
        }
    }

    pub fn empty() -> Self {
        Hybrid {
            rigid_bodies: vec![],
            articulated: vec![],
            deformables: vec![],
            cloths: vec![],
            halfspaces: vec![],
            controllers: vec![],
            gravity_enabled: true,
        }
    }

    pub fn disable_gravity(&mut self) {
        self.gravity_enabled = false;
    }

    pub fn add_rigid(&mut self, rigid: Rigid) {
        self.rigid_bodies.push(rigid);
    }

    pub fn add_articulated(&mut self, articulated: Articulated) {
        self.articulated.push(articulated);
        self.controllers
            .push(Box::new(NullArticulatedController {}));
    }

    pub fn add_deformable(&mut self, deformable: Deformable) {
        self.deformables.push(deformable);
    }

    pub fn add_cloth(&mut self, cloth: Cloth) {
        self.cloths.push(cloth);
    }

    pub fn add_halfspace(&mut self, halfspace: HalfSpace) {
        self.halfspaces.push(halfspace);
    }

    pub fn step(&mut self, dt: Float, input: &Vec<Float>) {
        let taus: Vec<DVector<Float>> = izip!(self.controllers.iter_mut(), self.articulated.iter())
            .map(|(c, a)| c.control(a, input))
            .collect();

        let v_rigids: Vec<DVector<Float>> = self
            .rigid_bodies
            .iter()
            .map(|b| b.free_velocity(dt))
            .collect();
        let v_articulated: Vec<DVector<Float>> =
            izip!(self.articulated.iter_mut(), taus.into_iter())
                .map(|(a, t)| a.free_velocity(dt, t, self.gravity_enabled))
                .collect();
        let v_deformables: Vec<DVector<Float>> = self
            .deformables
            .iter()
            .map(|b| b.free_velocity(dt, self.gravity_enabled))
            .collect();
        let v_cloths: Vec<DVector<Float>> =
            self.cloths.iter().map(|c| c.free_velocity(dt)).collect();
        let total_len = v_rigids.iter().map(|v| v.len()).sum::<usize>()
            + v_articulated.iter().map(|v| v.len()).sum::<usize>()
            + v_deformables.iter().map(|v| v.len()).sum::<usize>()
            + v_cloths.iter().map(|v| v.len()).sum::<usize>();
        let v_star: DVector<Float> = DVector::from_iterator(
            total_len,
            v_rigids
                .iter()
                .chain(v_articulated.iter())
                .chain(v_deformables.iter())
                .chain(v_cloths.iter())
                .flat_map(|v| v.data.as_vec().clone()),
        );

        let offset_articulated = self.rigid_bodies.len() * 6; // starting offset for deformable velocity within stacked total velocity vector
        let dof_articulated: usize = self.articulated.iter().map(|a| a.dof()).sum();
        let offset_deformable = offset_articulated + dof_articulated;

        let deformable_dof: usize = self.deformables.iter().map(|d| d.dof()).sum();
        let offset_cloth = offset_deformable + deformable_dof;

        let cloth_dof: usize = self.cloths.iter().map(|c| c.q.len()).sum();
        let total_dof = offset_cloth + cloth_dof;

        let mut Js: Vec<Matrix3xX<Float>> = vec![];
        let mu = 1.0; // friction coefficient

        // halfspace - deformable collision detection
        for halfspace in self.halfspaces.iter() {
            let n = &halfspace.normal;
            let mut i_deformable_offset = 0;
            for (i_deform, deformable) in self.deformables.iter().enumerate() {
                for (i_node, node) in deformable.get_positions().iter().enumerate() {
                    if halfspace.has_inside(node) {
                        let (t, b) = tangentials(n);
                        let C = Matrix3::from_rows(&[
                            1. / mu * n.transpose(),
                            t.transpose(),
                            b.transpose(),
                        ]);

                        // set jacobian for deformable part
                        let icol = offset_deformable + i_deformable_offset + i_node * 3;
                        let mut J = Matrix3xX::zeros(total_dof);
                        J.fixed_view_mut::<3, 3>(0, icol).copy_from(&C);

                        Js.push(J);
                    }
                }
                i_deformable_offset += deformable.dof();
            }
        }

        // rigid-deformable point-sphere collision detection
        for (i_rigid, rigid) in self.rigid_bodies.iter().enumerate() {
            let iso = rigid.pose.to_isometry();
            let translation = rigid.pose.translation;
            let T = compute_twist_transformation_matrix(&iso);

            let mut i_deformable_offset = 0;
            for (i_deform, deformable) in self.deformables.iter().enumerate() {
                for (i_node, pos) in deformable.get_positions().iter().enumerate() {
                    if (pos - translation).norm() <= 1.0 {
                        let n = UnitVector3::new_normalize(translation - pos);
                        let cp = pos;

                        let (t, b) = tangentials(&n);
                        let C = Matrix3::from_rows(&[
                            1. / mu * n.transpose(),
                            t.transpose(),
                            b.transpose(),
                        ]);

                        // set jacobian for deformable part
                        let icol = offset_deformable + i_deformable_offset + i_node * 3;
                        let mut J = Matrix3xX::zeros(total_dof);
                        J.fixed_view_mut::<3, 3>(0, icol).copy_from(&-C);

                        // set jacobian for rigid body part
                        let mut X = Matrix3x6::zeros();
                        let r = cp;
                        X.columns_mut(0, 3).copy_from(&-skew_symmetric(&r));
                        X.columns_mut(3, 3).copy_from(&Matrix3::identity());
                        J.fixed_view_mut::<3, 6>(0, i_rigid * 6)
                            .copy_from(&(C * X * T));
                        Js.push(J);
                    }
                }
                i_deformable_offset += deformable.dof();
            }
        }

        // articulated-deformable collision detection
        let mut icol_arti = offset_articulated;
        for (i_articulated, articulated) in self.articulated.iter().enumerate() {
            let v_art = &v_articulated[i_articulated];

            let body_twists = articulated.body_twists(v_art);

            let dof = articulated.dof();
            let offsets = articulated.offsets();
            let jacobians = articulated.jacobians();
            for (i_joint, (body, joint)) in
                izip!(articulated.bodies.iter(), articulated.joints.iter()).enumerate()
            {
                let mut i_deformable_offset = 0;
                for (i_deform, deformable) in self.deformables.iter().enumerate() {
                    let v_deform = &v_deformables[i_deform];
                    let contacts =
                        rigid_deformable_cd(body, deformable, &body_twists[i_joint], v_deform, dt);
                    for (cp, n, node_weights) in contacts.iter() {
                        let mut J = Matrix3xX::zeros(total_dof);

                        let (t, b) = tangentials(n);
                        let C = Matrix3::from_rows(&[
                            1. / mu * n.transpose(),
                            t.transpose(),
                            b.transpose(),
                        ]);

                        // set jacobian for deformable part
                        for (i_node, weight) in node_weights.iter() {
                            let icol = offset_deformable + i_deformable_offset + i_node * 3;
                            J.fixed_view_mut::<3, 3>(0, icol).copy_from(&(-weight * C));
                        }

                        // set jacobian for articulated body
                        let mut H = Matrix6xX::zeros(dof);
                        H.view_mut((0, offsets[i_joint]), (6, joint.dof()))
                            .copy_from(&jacobians[i_joint]);
                        let mut parent = i_joint;
                        while articulated.parents[parent] != parent {
                            parent = articulated.parents[parent];
                            H.view_mut((0, offsets[parent]), (6, articulated.joints[parent].dof()))
                                .copy_from(&jacobians[parent]);
                        }

                        let mut X = Matrix3x6::zeros();
                        let r = cp;
                        X.columns_mut(0, 3).copy_from(&-skew_symmetric(&r));
                        X.columns_mut(3, 3).copy_from(&Matrix3::identity());

                        J.view_mut((0, icol_arti), (3, dof)).copy_from(&(C * X * H));
                        Js.push(J);
                    }
                    i_deformable_offset += deformable.dof();
                }
            }
            icol_arti += dof;
        }

        // articulated-cloth collision detection
        let mut icol_arti = offset_articulated;
        for (i_art, articulated) in self.articulated.iter().enumerate() {
            let v_art = &v_articulated[i_art];

            let body_twists = articulated.body_twists(v_art);

            let dof = articulated.dof();
            let offsets = articulated.offsets();
            let jacobians = articulated.jacobians();
            for (i_joint, (body, joint)) in
                izip!(articulated.bodies.iter(), articulated.joints.iter()).enumerate()
            {
                let mut i_cloth_offset = 0;
                for (i_cloth, cloth) in self.cloths.iter().enumerate() {
                    let v_cloth = &v_cloths[i_cloth];
                    let contacts = rigid_cloth_ccd(body, cloth, &body_twists[i_joint], v_cloth, dt);
                    for (cp, n, node_weights) in contacts.iter() {
                        let mut J = Matrix3xX::zeros(total_dof);

                        let (t, b) = tangentials(n);
                        let C = Matrix3::from_rows(&[
                            1. / mu * n.transpose(),
                            t.transpose(),
                            b.transpose(),
                        ]);

                        // set jacobian for cloth part
                        for (i_node, weight) in node_weights.iter() {
                            let icol = offset_cloth + i_cloth_offset + i_node * 3;
                            J.fixed_view_mut::<3, 3>(0, icol).copy_from(&(-weight * C));
                        }

                        // set jacobian for articulated body
                        let mut H = Matrix6xX::zeros(dof);
                        H.view_mut((0, offsets[i_joint]), (6, joint.dof()))
                            .copy_from(&jacobians[i_joint]);
                        let mut parent = i_joint;
                        while articulated.parents[parent] != parent {
                            parent = articulated.parents[parent];
                            H.view_mut((0, offsets[parent]), (6, articulated.joints[parent].dof()))
                                .copy_from(&jacobians[parent]);
                        }

                        let mut X = Matrix3x6::zeros();
                        let r = cp;
                        X.columns_mut(0, 3).copy_from(&-skew_symmetric(&r));
                        X.columns_mut(3, 3).copy_from(&Matrix3::identity());

                        J.view_mut((0, icol_arti), (3, dof)).copy_from(&(C * X * H));
                        Js.push(J);
                    }
                    i_cloth_offset += cloth.dof();
                }
            }

            icol_arti += dof;
        }

        // deformable-deformable collision detection
        let mut d1_offset = 0;
        for (i1, d1) in self.deformables.iter().enumerate() {
            let v1 = &v_deformables[i1];
            let mut d2_offset = d1_offset + d1.dof();
            for (i2, d2) in self.deformables.iter().enumerate().skip(i1 + 1) {
                let v2 = &v_deformables[i2];
                let contacts = deformable_deformable_ccd(d1, d2, v1, v2, dt);
                for (cp, n, n1_ws, n2_ws) in contacts.iter() {
                    let mut J = Matrix3xX::zeros(total_dof);

                    let (t, b) = tangentials(n);
                    let C = Matrix3::from_rows(&[
                        1. / mu * n.transpose(),
                        t.transpose(),
                        b.transpose(),
                    ]);

                    // set jacobian for deformable1
                    for (i_node, weight) in n1_ws.iter() {
                        let icol = offset_deformable + d1_offset + i_node * 3;
                        J.fixed_view_mut::<3, 3>(0, icol).copy_from(&(-weight * C));
                    }

                    // set jacobian for deformable2
                    for (i_node, weight) in n2_ws.iter() {
                        let icol = offset_deformable + d2_offset + i_node * 3;
                        J.fixed_view_mut::<3, 3>(0, icol).copy_from(&(*weight * C));
                    }

                    Js.push(J);
                }
                d2_offset += d2.dof();
            }
            d1_offset += d1.dof();
        }

        let mut A: DMatrix<Float> = DMatrix::zeros(total_dof, total_dof);

        // Assemble A for rigid bodies
        let mut i = 0;
        for rigid in self.rigid_bodies.iter() {
            A.view_mut((i, i), (6, 6))
                .copy_from(&rigid.inertia.to_matrix());
            i += 6;
        }

        // Assemble A for articulated bodies
        let mut i = offset_articulated;
        for articulated in self.articulated.iter() {
            let dof = articulated.dof();
            A.view_mut((i, i), (dof, dof))
                .copy_from(&articulated.mass_matrix());
            i += dof;
        }

        // Assemble A for deformables
        let mut i = offset_deformable;
        for deformable in self.deformables.iter() {
            let dof = deformable.dof();
            A.view_mut((i, i), (dof, dof))
                .copy_from(&deformable.mass_matrix());
            i += dof;
        }

        // Assemble A for cloths
        let mut i = offset_cloth;
        for cloth in self.cloths.iter() {
            let dof = cloth.dof();
            A.view_mut((i, i), (dof, dof))
                .copy_from(&cloth.mass_matrix());
            i += dof;
        }

        // Solve convex optimization to resolve contact
        let P = CscMatrix::from(A.row_iter());
        let g = -v_star.transpose() * A;
        let q: Vec<Float> = Vec::from(g.as_slice());

        let A_ = if Js.len() > 0 {
            let mut rows: Vec<Matrix1xX<Float>> = vec![];
            for contact_J in Js.iter() {
                rows.extend(contact_J.row_iter().map(|r| r.into_owned()));
            }
            let J = DMatrix::from_rows(&rows);

            let mut J = CscMatrix::from(J.row_iter());
            J.scale(-1.);
            J
        } else {
            CscMatrix::zeros((0, total_dof))
        };
        let b = vec![0.; Js.len() * 3];
        let cones: Vec<SupportedConeT<Float>> = vec![SecondOrderConeT(3); Js.len()];

        let settings = DefaultSettingsBuilder::default()
            .verbose(false)
            .build()
            .unwrap();
        let mut solver = DefaultSolver::new(&P, &q, &A_, &b, &cones, settings);
        solver.solve();
        let v_sol = DVector::from(solver.solution.x);

        // Update rigid body velocities and poses
        let mut i = 0;
        for rigid in self.rigid_bodies.iter_mut() {
            let v_rigid = SpatialVector {
                angular: vector![v_sol[i], v_sol[i + 1], v_sol[i + 2]],
                linear: vector![v_sol[i + 3], v_sol[i + 4], v_sol[i + 5]],
            };
            rigid.twist = v_rigid;
            let qi = rigid.pose;
            let quaternion_dot = quaternion_derivative(&qi.rotation, &v_rigid.angular);
            let translation_dot = qi.rotation * v_rigid.linear;
            rigid.pose = Pose {
                translation: qi.translation + translation_dot * dt,
                rotation: UnitQuaternion::from_quaternion(
                    qi.rotation.quaternion() + quaternion_dot * dt,
                ),
            };
            i += 6;
        }

        // Update articulated velocities and poses
        let mut i = offset_articulated;
        for articulated in self.articulated.iter_mut() {
            let dof = articulated.dof();
            let v = v_sol.rows(i, dof).into_owned();
            articulated.integrate(v, dt);
            i += dof;
        }

        // Update deformable qdot and q
        let mut i = offset_deformable;
        for deformable in self.deformables.iter_mut() {
            let dof = deformable.dof();
            let v = v_sol.rows(i, dof).into_owned();
            deformable.q += &v * dt;
            deformable.qdot = v;
            i += dof;
        }

        // Update cloth qdot and q
        let mut i = offset_cloth;
        for cloth in self.cloths.iter_mut() {
            let dof = cloth.dof();
            let v = v_sol.rows(i, dof).into_owned();
            cloth.q += &v * dt;
            cloth.qdot = v;
            i += dof;
        }
    }

    pub fn set_rigid_poses(&mut self, poses: Vec<Pose>) {
        for (rigid, pose) in izip!(self.rigid_bodies.iter_mut(), poses.iter()) {
            rigid.pose = *pose;
        }
    }

    pub fn set_deformable_velocities(&mut self, vel: Vec<Vec<Vector3<Float>>>) {
        for (v, deformable) in izip!(vel.iter(), self.deformables.iter_mut()) {
            deformable.set_velocity(v.clone());
        }
    }

    pub fn set_rigid_velocities(&mut self, vel: Vec<Vector3<Float>>) {
        for (v, rigid) in izip!(vel.iter(), self.rigid_bodies.iter_mut()) {
            rigid.twist.linear = *v;
        }
    }

    pub fn set_controller(&mut self, i: usize, controller: impl ArticulatedController + 'static) {
        assert!(i < self.controllers.len());
        self.controllers[i] = Box::new(controller);
    }

    /// Computes total linear momentum
    pub fn linear_momentum(&self) -> Vector3<Float> {
        self.rigid_bodies
            .iter()
            .map(|b| b.linear_momentum())
            .sum::<Vector3<Float>>()
            + self
                .deformables
                .iter()
                .map(|b| b.linear_momentum())
                .sum::<Vector3<Float>>()
    }
}

#[cfg(test)]
mod hybrid_tests {
    use na::{vector, DVector, Vector3};

    use crate::{
        assert_vec_close, flog,
        hybrid::{
            articulated::Articulated,
            builders::{build_cube_cloth, build_gripper_cube, build_teddy},
            Deformable, Hybrid, Rigid,
        },
        joint::{Joint, JointVelocity},
        spatial::{pose::Pose, transform::Transform3D},
        util::read_file,
        WORLD_FRAME,
    };

    #[test]
    fn no_collision() {
        // Arrange
        let mut state = Hybrid::new_canonical();
        state.set_rigid_poses(vec![Pose::translation(vector![2.1, 0., 0.])]);
        let v_rigid = vector![1., 0., 0.];
        state.set_rigid_velocities(vec![v_rigid]);

        let v = vector![-1., 0., 0.];
        let v = vec![v, v, v, v];
        let qdot0 = DVector::from_iterator(
            state.deformables[0].q.len(),
            v.iter().flat_map(|x| x.iter().copied()),
        ); // TODO: add a util fn that flattens a Vec<Vector3>
        state.set_deformable_velocities(vec![v]);

        state.disable_gravity();

        // Act
        let final_time = 1.0;
        let dt = 1e-3;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            state.step(dt, &vec![]);
        }

        // Assert
        let rigid = &state.rigid_bodies[0];
        assert_vec_close!(rigid.twist.linear, v_rigid, 1e-3);
        assert_vec_close!(rigid.twist.angular, vector![0., 0., 0.], 1e-3);

        let deformable = &state.deformables[0];
        assert_vec_close!(&deformable.qdot, qdot0, 1e-3);
    }

    #[test]
    fn collision() {
        // Arrange
        let mut state = Hybrid::empty();
        state.add_rigid(Rigid::new_sphere(1., 1., "sphere"));
        state.set_rigid_poses(vec![Pose::translation(vector![2.5, 0., 0.])]);
        let v_rigid = vector![-1., 0., 0.];
        state.set_rigid_velocities(vec![v_rigid]);

        state.add_deformable(Deformable::new_octahedron());
        let v_deformable = vector![1. / 7., 0., 0.];
        let v = vec![v_deformable; 7];
        state.set_deformable_velocities(vec![v]);

        state.disable_gravity();

        let linear_momentum = state.linear_momentum();

        // Act
        let final_time = 1.0;
        let dt = 1e-3;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            state.step(dt, &vec![]);
        }

        // Assert
        let rigid = &state.rigid_bodies[0];
        assert!((rigid.twist.linear - v_rigid).x > 0.);

        let deformable = &state.deformables[0];
        for vel in deformable.get_velocities() {
            assert!((vel - v_deformable).x < 0.);
        }

        assert_vec_close!(state.linear_momentum(), linear_momentum, 1e-3);
    }

    #[test]
    fn two_carts_squashing_cube() {
        // Arrange
        let mut state = Hybrid::empty();

        let l_cube = 1.0;
        let cart_offset = 0.0; // Note: this is useful for making sure contact point is not on mesh edge. Collision-handling is not exactly penetration free because satisfying the contact constraint for contact point and normal at current step, does not ensure no penetration after velocity integration, because nearest point and normal could change.
                               // TODO(contact): make the test work without offset

        let m = 1.0;
        let w = 1.0;
        let d = 0.1;
        let cart_frame = "cart";
        let cart = Rigid::new_cuboid_at(&vector![0., 0., 0.], m, w, d, d, cart_frame);
        let cart_to_world = Transform3D::move_xyz(
            cart_frame,
            WORLD_FRAME,
            l_cube + 0.5 + w / 2.,
            cart_offset,
            0.,
        );

        let bodies = vec![cart];
        let joints = vec![Joint::new_prismatic(cart_to_world, Vector3::x_axis())];
        let mut articulated = Articulated::new(bodies, joints);
        articulated.set_joint_v(0, JointVelocity::Float(-1.0));
        state.add_articulated(articulated);

        let cart2_frame = "cart2";
        let cart2 = Rigid::new_cuboid_at(&vector![0., 0., 0.], m, w, d, d, cart2_frame);
        let cart2_to_world =
            Transform3D::move_xyz(cart2_frame, WORLD_FRAME, -0.5 - w / 2., cart_offset, 0.);
        let bodies = vec![cart2];
        let joints = vec![Joint::new_prismatic(cart2_to_world, Vector3::x_axis())];
        let mut articulated = Articulated::new(bodies, joints);
        articulated.set_joint_v(0, JointVelocity::Float(1.0));
        state.add_articulated(articulated);

        state.add_deformable(Deformable::new_cube(1e2)); // k = 1e2
        state.disable_gravity();

        // Act
        let final_time = 2.0;
        let dt = 1e-3;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            state.step(dt, &vec![]);
        }

        // Assert
        let v_cart = state.articulated[0].v()[0];
        let v_cart2 = state.articulated[1].v()[0];
        // assert_close!(v_cart, -v_cart2, 1e-2); // TODO: test under no friction
        assert!(v_cart > 0.);
        assert!(v_cart2 < 0.);
        let vs = state.deformables[0].get_velocities();
        for v in vs.iter() {
            assert_vec_close!(v, [0.; 3], 2e-2); // set tol to 1e-2, under no friction
        }
    }

    #[test]
    fn cube_cloth() {
        // Arrange
        let mut state = build_cube_cloth();

        // Act
        let final_time = 0.11;
        let dt = 1e-3;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            state.step(dt, &vec![0., 0., 0.]);
        }
    }

    #[ignore] // brittle, depends on dt
    #[test]
    fn gripper() {
        // Arrange
        let mut state = build_gripper_cube();

        // Act
        let final_time = 0.5;
        let dt = 1e-3;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            state.step(dt, &vec![0., 0., 0.]);
        }
    }

    #[test]
    fn teddy() {
        // Arrange
        let buf = read_file("data/teddy.vtk");
        let mut state = build_teddy(&buf);

        // Act
        let final_time = 0.1;
        let dt = 1e-3;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            state.step(dt, &vec![0., 0., 0.]);
        }
    }

    #[ignore] // TODO: complete this test
    #[test]
    fn double_pendulum_and_cube() {
        // Arrange
        let mut state = Hybrid::empty();
        let l = 1.0;
        let r = 0.1;
        let pendulum_frame = "pendulum";
        let pendulum2_frame = "pendulum2";
        let bodies = vec![
            Rigid::new_sphere_at(&vector![-l, 0., 0.], 1., r, pendulum_frame),
            Rigid::new_sphere_at(&vector![0., 0., l], 1., r, pendulum2_frame),
        ];
        let pendulum_to_world = Transform3D::move_x(pendulum_frame, WORLD_FRAME, -l);
        let pendulum2_to_pendulum = Transform3D::move_x(pendulum2_frame, pendulum_frame, -l);

        let joints = vec![
            Joint::new_revolute(pendulum_to_world, Vector3::y_axis()),
            Joint::new_revolute(pendulum2_to_pendulum, Vector3::y_axis()),
        ];
        state.add_articulated(Articulated::new(bodies, joints));

        state.add_deformable(Deformable::new_cube(1e2));

        // Act
        let final_time = 1.4;
        let dt = 1e-3;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            state.step(dt, &vec![]);
        }

        // Assert
    }
}
