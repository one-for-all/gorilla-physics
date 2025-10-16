use clarabel::{
    algebra::{CscMatrix, MatrixMathMut},
    solver::{
        DefaultSettingsBuilder, DefaultSolver, IPSolver,
        SupportedConeT::{self, NonnegativeConeT},
    },
};
use itertools::izip;
use na::{
    vector, DMatrix, DVector, Matrix1xX, Matrix3, Matrix3x6, Matrix6xX, UnitQuaternion,
    UnitVector3, Vector3,
};

use crate::{
    flog,
    hybrid::{articulated::Articulated, visual::Visual},
    spatial::{
        pose::Pose, spatial_vector::SpatialVector, twist::compute_twist_transformation_matrix,
    },
    types::Float,
    util::{quaternion_derivative, skew_symmetric},
};

pub use deformable::Deformable;
pub use rigid::Rigid;

pub mod articulated;
pub mod deformable;
pub mod rigid;
pub mod visual;

pub struct Hybrid {
    pub rigid_bodies: Vec<Rigid>,
    pub articulated: Vec<Articulated>,
    pub deformables: Vec<Deformable>,
}

impl Hybrid {
    /// A canonical hybrid system with 1 sphere rigid-body and 1 tetrahedron deformable
    pub fn new_canonical() -> Self {
        let rigid = Rigid::new_sphere();
        let deformable = Deformable::new_tetrahedron();

        Hybrid {
            rigid_bodies: vec![rigid],
            articulated: vec![],
            deformables: vec![deformable],
        }
    }

    pub fn empty() -> Self {
        Hybrid {
            rigid_bodies: vec![],
            articulated: vec![],
            deformables: vec![],
        }
    }

    pub fn add_rigid(&mut self, rigid: Rigid) {
        self.rigid_bodies.push(rigid);
    }

    pub fn add_articulated(&mut self, articulated: Articulated) {
        self.articulated.push(articulated);
    }

    pub fn add_deformable(&mut self, deformable: Deformable) {
        self.deformables.push(deformable);
    }

    pub fn step(&mut self, dt: Float) {
        let deformable = &self.deformables[0];

        let v_rigids: Vec<DVector<Float>> = self
            .rigid_bodies
            .iter()
            .map(|b| b.free_velocity(dt))
            .collect();
        let v_articulated: Vec<DVector<Float>> = self
            .articulated
            .iter_mut()
            .map(|a| a.free_velocity(dt))
            .collect();
        let v_deformables: Vec<DVector<Float>> = self
            .deformables
            .iter()
            .map(|b| b.free_velocity(dt))
            .collect();
        let total_len = v_rigids.iter().map(|v| v.len()).sum::<usize>()
            + v_articulated.iter().map(|v| v.len()).sum::<usize>()
            + v_deformables.iter().map(|v| v.len()).sum::<usize>();
        let v_star: DVector<Float> = DVector::from_iterator(
            total_len,
            v_rigids
                .iter()
                .chain(v_articulated.iter())
                .chain(v_deformables.iter())
                .flat_map(|v| v.data.as_vec().clone()),
        );

        let offset_articulated = self.rigid_bodies.len() * 6; // starting offset for deformable velocity within stacked total velocity vector
        let dof_articulated: usize = self.articulated.iter().map(|a| a.dof()).sum();
        let offset_deformable = offset_articulated + dof_articulated;

        let deformable_dof = deformable.q.len();
        let total_dof = offset_deformable + deformable_dof;

        let mut Js: Vec<Matrix1xX<Float>> = vec![];

        // rigid-deformable point-sphere collision detection
        for (i_rigid, rigid) in self.rigid_bodies.iter().enumerate() {
            let iso = rigid.pose.to_isometry();
            let translation = rigid.pose.translation;
            let T = compute_twist_transformation_matrix(&iso);

            for (i_node, pos) in deformable.get_positions().iter().enumerate() {
                if (pos - translation).norm() <= 1.0 {
                    let n = UnitVector3::new_normalize(pos - translation);
                    let cp = pos;

                    // set jacobian for deformable part
                    let icol = offset_deformable + i_node * 3;
                    let mut J = Matrix1xX::zeros(offset_deformable + deformable.q.len());
                    J.fixed_view_mut::<1, 3>(0, icol).copy_from(&n.transpose());

                    // set jacobian for rigid body part
                    let mut X = Matrix3x6::zeros();
                    let r = cp;
                    X.columns_mut(0, 3).copy_from(&-skew_symmetric(&r));
                    X.columns_mut(3, 3).copy_from(&Matrix3::identity());
                    J.fixed_view_mut::<1, 6>(0, i_rigid * 6)
                        .copy_from(&(-n.transpose() * X * T));
                    Js.push(J);
                }
            }
        }

        // articulated-deformable collision detection
        let mut icol_arti = offset_articulated;
        for (i_articulated, articulated) in self.articulated.iter().enumerate() {
            let dof = articulated.dof();
            let offsets = articulated.offsets();
            let jacobians = articulated.jacobians();
            for (i_joint, (body, joint)) in
                izip!(articulated.bodies.iter(), articulated.joints.iter()).enumerate()
            {
                for (collider, iso_collider_to_body) in body.visual.iter() {
                    let iso = body.pose.to_isometry() * iso_collider_to_body;
                    let collider_pos = iso.translation.vector;

                    for (i_node, node_pos) in deformable.get_positions().iter().enumerate() {
                        match collider {
                            Visual::Sphere(sphere) => {
                                if (node_pos - collider_pos).norm() < sphere.r {
                                    let n = UnitVector3::new_normalize(collider_pos - node_pos);
                                    let cp = node_pos;

                                    let mut J = Matrix1xX::zeros(total_dof);

                                    // set jacobian for deformable part
                                    let icol = offset_deformable + i_node * 3;
                                    J.fixed_view_mut::<1, 3>(0, icol).copy_from(&-n.transpose());

                                    // set jacobian for articulated body
                                    let mut H = Matrix6xX::zeros(dof);
                                    H.view_mut((0, offsets[i_joint]), (6, joint.dof()))
                                        .copy_from(&jacobians[i_joint]);
                                    let mut parent = i_joint;
                                    while articulated.parents[parent] != parent {
                                        parent = articulated.parents[parent];
                                        H.view_mut(
                                            (0, offsets[parent]),
                                            (6, articulated.joints[parent].dof()),
                                        )
                                        .copy_from(&jacobians[parent]);
                                    }

                                    let mut X = Matrix3x6::zeros();
                                    let r = cp;
                                    X.columns_mut(0, 3).copy_from(&-skew_symmetric(&r));
                                    X.columns_mut(3, 3).copy_from(&Matrix3::identity());

                                    J.view_mut((0, icol_arti), (1, dof))
                                        .copy_from(&(n.transpose() * X * H));
                                    Js.push(J);
                                }
                            }
                        }
                    }
                }
            }
            icol_arti += dof;
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
        A.view_mut(
            (offset_deformable, offset_deformable),
            (deformable_dof, deformable_dof),
        )
        .copy_from(&DMatrix::identity(deformable_dof, deformable_dof));

        // Solve convex optimization to resolve contact
        let P = CscMatrix::from(A.row_iter());
        let g = -v_star.transpose() * A;
        let q: Vec<Float> = Vec::from(g.as_slice());

        let A_ = if Js.len() > 0 {
            let J = DMatrix::from_rows(&Js);
            let mut J = CscMatrix::from(J.row_iter());
            J.scale(-1.);
            J
        } else {
            CscMatrix::zeros((0, total_dof))
        };
        let b = vec![0.; Js.len()];
        let cones: Vec<SupportedConeT<Float>> = vec![NonnegativeConeT(Js.len())];

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
        let v_deformable = v_sol.rows(offset_deformable, deformable_dof).into_owned();
        let deformable = &mut self.deformables[0];
        deformable.q += &v_deformable * dt;
        deformable.qdot = v_deformable;
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
        assert_vec_close,
        hybrid::{articulated::Articulated, Deformable, Hybrid, Rigid},
        joint::Joint,
        spatial::{pose::Pose, transform::Transform3D},
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

        // Act
        let final_time = 1.0;
        let dt = 1e-3;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            state.step(dt);
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
        state.add_rigid(Rigid::new_sphere());
        state.set_rigid_poses(vec![Pose::translation(vector![2.5, 0., 0.])]);
        let v_rigid = vector![-1., 0., 0.];
        state.set_rigid_velocities(vec![v_rigid]);

        state.add_deformable(Deformable::new_octahedron());
        let v_deformable = vector![1. / 7., 0., 0.];
        let v = vec![v_deformable; 7];
        state.set_deformable_velocities(vec![v]);

        let linear_momentum = state.linear_momentum();

        // Act
        let final_time = 1.0;
        let dt = 1e-3;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            state.step(dt);
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

        state.add_deformable(Deformable::new_cube());

        // Act
        let final_time = 1.4;
        let dt = 1e-3;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            state.step(dt);
        }

        // Assert
    }
}
