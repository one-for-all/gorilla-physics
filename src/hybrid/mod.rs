use clarabel::{
    algebra::{CscMatrix, MatrixMathMut},
    solver::{
        DefaultSettingsBuilder, DefaultSolver, IPSolver,
        SupportedConeT::{self, NonnegativeConeT},
    },
};
use itertools::izip;
use na::{
    dvector, vector, DMatrix, DVector, Matrix1xX, Matrix3, Matrix3x6, UnitQuaternion, UnitVector3,
    Vector3,
};
use nalgebra_sparse::{CooMatrix, CsrMatrix};

use crate::{
    inertia::SpatialInertia,
    spatial::{
        pose::Pose, spatial_vector::SpatialVector, twist::compute_twist_transformation_matrix,
    },
    types::Float,
    util::{quaternion_derivative, skew_symmetric},
};

/// Rigid body
pub struct Rigid {
    pub inertia: SpatialInertia,
    pub pose: Pose,
    pub vel: SpatialVector,
}

impl Rigid {
    pub fn new_sphere() -> Self {
        let m = 1.0;
        let r = 1.0;
        let moment = 2. / 5. * m * r * r;
        let moment = Matrix3::from_diagonal_element(moment);
        let cross = Vector3::zeros();
        let inertia = SpatialInertia::new(moment, cross, m, "sphere");
        Rigid {
            inertia,
            pose: Pose::identity(),
            vel: SpatialVector::zero(),
        }
    }

    /// free-motion velocity in body frame
    pub fn free_velocity(&self, dt: Float) -> DVector<Float> {
        let mut v_free = dvector![];
        v_free.extend(self.vel.angular.iter().cloned());
        v_free.extend(self.vel.linear.iter().cloned());
        assert_eq!(v_free.len(), 6);
        v_free
    }

    /// Computes linear momentum in world frame
    pub fn linear_momentum(&self) -> Vector3<Float> {
        let linear =
            self.inertia.mass * self.vel.linear - self.inertia.cross_part.cross(&self.vel.angular);
        self.pose.rotation * linear
    }
}

/// Mass-spring modeled deformable
pub struct Deformable {
    pub nodes: Vec<Vector3<Float>>,
    pub tetrahedra: Vec<Vec<usize>>,

    pub faces: Vec<[usize; 3]>,

    pub q: DVector<Float>,
    pub qdot: DVector<Float>,
}

impl Deformable {
    pub fn new(nodes: Vec<Vector3<Float>>, tetrahedra: Vec<Vec<usize>>) -> Self {
        let dof = nodes.len() * 3;
        let q = DVector::from_iterator(dof, nodes.iter().flat_map(|x| x.iter().copied()));
        let qdot = DVector::zeros(q.len());

        let faces = Self::compute_boundary_facets(&tetrahedra);

        Deformable {
            nodes,
            tetrahedra,
            faces,
            q,
            qdot,
        }
    }

    pub fn new_tetrahedron() -> Self {
        let nodes = vec![
            vector![0., 0., 0.],
            vector![1., 0., 0.],
            vector![0., 1., 0.],
            vector![0., 0., 1.],
        ];
        let tetrahedra = vec![vec![0, 1, 2, 3]];

        let dof = nodes.len() * 3;
        let q = DVector::from_iterator(dof, nodes.iter().flat_map(|x| x.iter().copied()));
        let qdot = DVector::zeros(q.len());

        let faces = Self::compute_boundary_facets(&tetrahedra);

        Deformable {
            nodes,
            tetrahedra,
            faces,
            q,
            qdot,
        }
    }

    pub fn new_pyramid() -> Self {
        let nodes = vec![
            vector![0., 0., 0.],
            vector![1., 0., 0.],
            vector![0., 1., 0.],
            vector![0., 0., 1.],
            vector![-1., 0., 0.], // right point
            vector![0., -1., 0.], // front point
        ];
        let tetrahedra = vec![
            vec![0, 1, 2, 3],
            vec![0, 2, 4, 3],
            vec![0, 1, 3, 5],
            vec![0, 3, 4, 5],
        ];
        Self::new(nodes, tetrahedra)
    }

    pub fn new_octahedron() -> Self {
        let nodes = vec![
            vector![0., 0., 0.],
            vector![1., 0., 0.],
            vector![0., 1., 0.],
            vector![0., 0., 1.],
            vector![-1., 0., 0.], // left point
            vector![0., -1., 0.], // front point
            vector![0., 0., -1.], // bottom point
        ];
        let tetrahedra = vec![
            // top half
            vec![0, 1, 2, 3],
            vec![0, 2, 4, 3],
            vec![0, 1, 3, 5],
            vec![0, 3, 4, 5],
            // bottom half
            vec![0, 2, 1, 6],
            vec![0, 4, 2, 6],
            vec![0, 1, 5, 6],
            vec![0, 5, 4, 6],
        ];
        Self::new(nodes, tetrahedra)
    }

    pub fn new_cube() -> Self {
        let nodes = vec![
            vector![0., 0., 0.],
            vector![1., 0., 0.],
            vector![0., 1., 0.],
            vector![0., 0., 1.],
            vector![1., 1., 0.], // base back-right
            vector![1., 1., 1.], // top back-right
            vector![1., 0., 1.], // top front-right
            vector![0., 1., 1.], // top back-left
        ];
        let tetrahedra = vec![
            vec![0, 1, 2, 3], // bottom front
            vec![4, 2, 1, 5], // bottom back
            vec![3, 6, 1, 5], // front right
            vec![7, 3, 2, 5], // back left
            vec![3, 5, 1, 2], // center tetra
        ];
        Self::new(nodes, tetrahedra)
    }

    /// Linear momentum assuming mass = 1
    pub fn linear_momentum(&self) -> Vector3<Float> {
        self.get_velocities()
            .iter()
            .fold(Vector3::<Float>::zeros(), |acc, v| acc + v)
    }

    pub fn get_positions(&self) -> Vec<Vector3<Float>> {
        let mut p = vec![];
        let mut i = 0;
        while i < self.q.len() {
            p.push(vector![self.q[i], self.q[i + 1], self.q[i + 2]]);
            i += 3;
        }
        p
    }

    pub fn get_velocities(&self) -> Vec<Vector3<Float>> {
        let mut v = vec![];
        let mut i = 0;
        while i < self.q.len() {
            v.push(vector![self.qdot[i], self.qdot[i + 1], self.qdot[i + 2]]);
            i += 3;
        }
        v
    }

    pub fn set_velocity(&mut self, v: Vec<Vector3<Float>>) {
        self.qdot = DVector::from_iterator(self.q.len(), v.iter().flat_map(|x| x.iter().copied()));
    }

    /// free-motion velocity after timestep
    pub fn free_velocity(&self, dt: Float) -> DVector<Float> {
        let n_nodes = self.nodes.len();
        let mut adj: CooMatrix<u8> = CooMatrix::new(n_nodes, n_nodes);
        for tetrahedron in self.tetrahedra.iter() {
            for i in 0..tetrahedron.len() - 1 {
                for j in (i + 1)..tetrahedron.len() {
                    let v0 = tetrahedron[i];
                    let v1 = tetrahedron[j];
                    adj.push(v0, v1, 1);
                    adj.push(v1, v0, 1);
                }
            }
        }
        let adj = CsrMatrix::from(&adj);

        let mut edges: Vec<(usize, usize)> = vec![];
        let mut rs: Vec<Float> = vec![]; // rest lengths
        for irow in 0..n_nodes {
            let row = adj.row(irow);
            for icol in row.col_indices() {
                let icol = *icol;
                if irow < icol {
                    assert!(!edges.contains(&(irow, icol)));
                    edges.push((irow, icol));
                    rs.push((self.nodes[irow] - self.nodes[icol]).norm());
                }
            }
        }

        let dof = self.q.len();
        let mut f_elastic: DVector<Float> = DVector::zeros(dof);
        for (edge, r) in izip!(edges.iter(), rs.iter()) {
            let (n0, n1) = edge;
            let i_n0 = n0 * 3;
            let i_n1 = n1 * 3;
            let q0 = vector![self.q[i_n0], self.q[i_n0 + 1], self.q[i_n0 + 2]];
            let q1 = vector![self.q[i_n1], self.q[i_n1 + 1], self.q[i_n1 + 2]];

            let q0q1 = q1 - q0;
            let l = q0q1.norm();

            let k = 1e5;
            let f_n0 = k * (l - r) * q0q1 / l;
            let f_n1 = -f_n0;

            for i in 0..3 {
                f_elastic[i_n0 + i] += f_n0[i];
                f_elastic[i_n1 + i] += f_n1[i];
            }
        }

        let mut f_damping: DVector<Float> = DVector::zeros(dof);
        for edge in edges.iter() {
            let (n0, n1) = edge;
            let i_n0 = n0 * 3;
            let i_n1 = n1 * 3;
            let q0 = vector![self.q[i_n0], self.q[i_n0 + 1], self.q[i_n0 + 2]];
            let q1 = vector![self.q[i_n1], self.q[i_n1 + 1], self.q[i_n1 + 2]];

            let q0q1 = q1 - q0;
            let l = q0q1.norm();
            let unit_q0q1 = q0q1 / l;

            let qdot0 = vector![self.qdot[i_n0], self.qdot[i_n0 + 1], self.qdot[i_n0 + 2]];
            let qdot1 = vector![self.qdot[i_n1], self.qdot[i_n1 + 1], self.qdot[i_n1 + 2]];
            let qdot0qdot1 = qdot1 - qdot0;

            let b = 20.;
            let f_n0 = b * qdot0qdot1.dot(&unit_q0q1) * unit_q0q1;
            let f_n1 = -f_n0;

            for i in 0..3 {
                f_damping[i_n0 + i] += f_n0[i];
                f_damping[i_n1 + i] += f_n1[i];
            }
        }

        let f_total = f_elastic + f_damping;
        let m_v0 = -dt * f_total; // momentum residual with velocity at t0

        let n_dim = self.q.len();
        let M: DMatrix<Float> = DMatrix::identity(n_dim, n_dim); // mass matrix
        let A = M;
        let v0 = &self.qdot;
        let v_star = v0 - A.clone().try_inverse().unwrap() * m_v0;

        v_star
    }

    /// Extract the boundary facets, for better visualization
    /// TODO: extract into a global function
    pub fn compute_boundary_facets(tetrahedra: &Vec<Vec<usize>>) -> Vec<[usize; 3]> {
        let mut facets = vec![];
        for tetrahedron in tetrahedra.iter() {
            let v0 = tetrahedron[0];
            let v1 = tetrahedron[1];
            let v2 = tetrahedron[2];
            let v3 = tetrahedron[3];

            facets.push({
                let v = [v1, v2, v3];
                v
            });
            facets.push({
                let v = [v0, v3, v2];
                v
            });
            facets.push({
                let v = [v0, v1, v3];
                v
            });
            facets.push({
                let v = [v0, v2, v1];
                v
            });
        }
        facets.sort_by(|a, b| {
            let mut a_clone = a.clone();
            a_clone.sort();
            let mut b_clone = b.clone();
            b_clone.sort();

            a_clone.cmp(&b_clone)
        });

        // Get the faces that only appear once. They are the boundary faces.
        let mut boundary_facets = vec![];
        for i in 0..facets.len() {
            let cur = facets[i];
            let mut cur_sort = cur.clone();
            cur_sort.sort();

            if i > 0 {
                let mut prev_sort = facets[i - 1].clone();
                prev_sort.sort();
                if cur_sort == prev_sort {
                    continue;
                }
            }

            if i < facets.len() - 1 {
                let mut next_sort = facets[i + 1].clone();
                next_sort.sort();
                if cur_sort == next_sort {
                    continue;
                }
            }
            boundary_facets.push(cur);
        }

        boundary_facets
    }
}

pub struct Hybrid {
    pub rigid_bodies: Vec<Rigid>,
    pub deformables: Vec<Deformable>,
}

impl Hybrid {
    /// A canonical hybrid system with 1 sphere rigid-body and 1 tetrahedron deformable
    pub fn new_canonical() -> Self {
        let rigid = Rigid::new_sphere();
        let deformable = Deformable::new_tetrahedron();

        Hybrid {
            rigid_bodies: vec![rigid],
            deformables: vec![deformable],
        }
    }

    pub fn empty() -> Self {
        Hybrid {
            rigid_bodies: vec![],
            deformables: vec![],
        }
    }

    pub fn add_rigid(&mut self, rigid: Rigid) {
        self.rigid_bodies.push(rigid);
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
        let v_deformables: Vec<DVector<Float>> = self
            .deformables
            .iter()
            .map(|b| b.free_velocity(dt))
            .collect();
        let total_len = v_rigids.iter().map(|v| v.len()).sum::<usize>()
            + v_deformables.iter().map(|v| v.len()).sum::<usize>();
        let v_star: DVector<Float> = DVector::from_iterator(
            total_len,
            v_rigids
                .iter()
                .chain(v_deformables.iter())
                .flat_map(|v| v.data.as_vec().clone()),
        );

        let offset_deformable = self.rigid_bodies.len() * 6; // starting offset for deformable velocity within stacked total velocity vector

        // rigid-deformable point-sphere collision detection
        let mut Js: Vec<Matrix1xX<Float>> = vec![];
        for (i_rigid, rigid) in self.rigid_bodies.iter().enumerate() {
            let iso = rigid.pose.to_isometry();
            let translation = rigid.pose.translation;
            let T = compute_twist_transformation_matrix(&iso);

            for (i_node, pos) in deformable.get_positions().iter().enumerate() {
                if (pos - translation).norm() <= 1.0 {
                    let n = UnitVector3::new_normalize(pos - translation);
                    let cp = pos;

                    let icol = offset_deformable + i_node * 3;
                    let mut J = Matrix1xX::zeros(offset_deformable + deformable.q.len());
                    J.fixed_view_mut::<1, 3>(0, icol).copy_from(&n.transpose());

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

        let deformable_dof = deformable.q.len();
        let dof = offset_deformable + deformable_dof;
        let mut A: DMatrix<Float> = DMatrix::zeros(dof, dof);
        A.view_mut(
            (offset_deformable, offset_deformable),
            (deformable_dof, deformable_dof),
        )
        .copy_from(&DMatrix::identity(deformable_dof, deformable_dof));

        let mut i = 0;
        for rigid in self.rigid_bodies.iter() {
            A.view_mut((i, i), (6, 6))
                .copy_from(&rigid.inertia.to_matrix());
            i += 6;
        }

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
            CscMatrix::zeros((0, dof))
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
            rigid.vel = v_rigid;
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
            rigid.vel.linear = *v;
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
    use na::{vector, DVector};

    use crate::{
        assert_vec_close, flog,
        hybrid::{Deformable, Hybrid, Rigid},
        spatial::pose::Pose,
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
        assert_vec_close!(rigid.vel.linear, v_rigid, 1e-3);
        assert_vec_close!(rigid.vel.angular, vector![0., 0., 0.], 1e-3);

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
        assert!((rigid.vel.linear - v_rigid).x > 0.);

        let deformable = &state.deformables[0];
        for vel in deformable.get_velocities() {
            assert!((vel - v_deformable).x < 0.);
        }

        assert_vec_close!(state.linear_momentum(), linear_momentum, 1e-3);
    }
}
