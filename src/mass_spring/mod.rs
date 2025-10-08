use clarabel::{
    algebra::{CscMatrix, MatrixMathMut},
    solver::{
        DefaultSettingsBuilder, DefaultSolution, DefaultSolver, IPSolver,
        SupportedConeT::{self, NonnegativeConeT},
    },
};
use itertools::izip;
use na::{vector, DMatrix, DVector, Matrix1xX, Matrix3, UnitVector3, Vector3};
use nalgebra_sparse::{CooMatrix, CsrMatrix};

use crate::{flog, types::Float};

pub struct MassSpring {
    pub nodes: Vec<Vector3<Float>>,
    pub tetrahedra: Vec<Vec<usize>>,

    pub dof: usize, // degree of freedom of system. dof == q.len() == qdot.len().

    pub faces: Vec<[usize; 3]>,

    pub q: DVector<Float>,
    pub qdot: DVector<Float>,
}

impl MassSpring {
    pub fn new(nodes: Vec<Vector3<Float>>, tetrahedra: Vec<Vec<usize>>) -> Self {
        let dof = nodes.len() * 3;
        let q = DVector::from_iterator(dof, nodes.iter().flat_map(|x| x.iter().copied()));
        let qdot = DVector::zeros(q.len());

        let faces = MassSpring::compute_boundary_facets(&tetrahedra);

        MassSpring {
            nodes,
            tetrahedra,
            dof,
            faces,
            q,
            qdot,
        }
    }

    pub fn unit_tetrahedron() -> Self {
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

        let faces = MassSpring::compute_boundary_facets(&tetrahedra);

        MassSpring {
            nodes,
            tetrahedra,
            dof,
            faces,
            q,
            qdot,
        }
    }

    pub fn set_velocity(&mut self, v: Vec<Vector3<Float>>) {
        self.qdot = DVector::from_iterator(self.dof, v.iter().flat_map(|x| x.iter().copied()));
    }

    pub fn get_velocity(&self) -> Vec<Vector3<Float>> {
        let mut v = vec![];
        let mut i = 0;
        while i < self.dof {
            v.push(vector![self.qdot[i], self.qdot[i + 1], self.qdot[i + 2]]);
            i += 3;
        }
        v
    }

    pub fn set_positions(&mut self, p: Vec<Vector3<Float>>) {
        self.q = DVector::from_iterator(self.dof, p.iter().flat_map(|x| x.iter().copied()));
    }

    pub fn get_positions(&self) -> Vec<Vector3<Float>> {
        let mut p = vec![];
        let mut i = 0;
        while i < self.dof {
            p.push(vector![self.q[i], self.q[i + 1], self.q[i + 2]]);
            i += 3;
        }
        p
    }

    /// center of mass at current q
    pub fn com(&self) -> Vector3<Float> {
        self.get_positions().iter().sum::<Vector3<Float>>() / self.nodes.len() as Float
    }

    /// Angular momentum assuming mass = 1
    pub fn angular_momentum(&self) -> Vector3<Float> {
        izip!(self.get_positions().iter(), self.get_velocity().iter())
            .fold(Vector3::<Float>::zeros(), |acc, (p, v)| acc + p.cross(&v))
    }

    /// Linear momentum assuming mass = 1
    pub fn linear_momentum(&self) -> Vector3<Float> {
        self.get_velocity()
            .iter()
            .fold(Vector3::<Float>::zeros(), |acc, v| acc + v)
    }

    pub fn step(&mut self, dt: Float) {
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
                    edges.push((irow, icol));
                    rs.push((self.nodes[irow] - self.nodes[icol]).norm());
                }
            }
        }

        let mut f_elastic: DVector<Float> = DVector::zeros(self.q.len());
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

        let mut f_damping: DVector<Float> = DVector::zeros(self.dof);
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

        let mut f_gravity = DVector::zeros(self.dof);
        let mut i = 0;
        while i < self.dof {
            f_gravity[i + 2] = -9.8;
            i += 3;
        }

        let f_total = f_elastic + f_damping + f_gravity;
        let m_v0 = -dt * f_total; // momentum residual with velocity at t0

        let n_dim = self.q.len();
        let M: DMatrix<Float> = DMatrix::identity(n_dim, n_dim); // mass matrix
        let A = M;
        let v0 = &self.qdot;
        let v_star = v0 - A.clone().try_inverse().unwrap() * m_v0;

        // Solve convex optimization to resolve contact
        let P = CscMatrix::from(A.row_iter());
        let g = -v_star.transpose() * A;
        let q: Vec<Float> = Vec::from(g.as_slice());

        let mut Js: Vec<Matrix1xX<Float>> = vec![];
        let n = vector![0., 0., 1.];
        for (i, pos) in self.get_positions().iter().enumerate() {
            if pos.z <= 0. {
                // collision
                let icol = i * 3;
                let mut S = DMatrix::zeros(3, self.dof);
                S.fixed_view_mut::<3, 3>(0, icol)
                    .copy_from(&Matrix3::identity());
                Js.push(n.transpose() * S);
            }
        }

        let A_ = if Js.len() > 0 {
            let J = DMatrix::from_rows(&Js);
            let mut J = CscMatrix::from(J.row_iter());
            J.scale(-1.);
            J
        } else {
            CscMatrix::zeros((0, self.dof))
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

        self.qdot = v_sol;
        self.q += &self.qdot * dt;
    }

    /// Extract the boundary facets, for better visualization
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

#[cfg(test)]
mod mass_spring_tests {
    use itertools::izip;
    use na::{dvector, vector, DVector, Vector3};
    use rand::rng;

    use crate::{
        assert_vec_close, flog, mass_spring::MassSpring, types::Float,
        util::test_utils::random_vector3,
    };

    #[test]
    fn stationary() {
        // Arrange
        let mut deformable = MassSpring::unit_tetrahedron();
        let q0 = deformable.q.clone();
        let qdot0 = deformable.qdot.clone();

        // Act
        let final_time = 1.0;
        let dt = 1e-3;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            deformable.step(dt);
        }

        // Assert
        assert_vec_close!(q0, deformable.q, 1e-3);
        assert_vec_close!(qdot0, deformable.qdot, 1e-3);
    }

    #[test]
    fn translating() {
        // Arrange
        let mut deformable = MassSpring::unit_tetrahedron();
        let q0 = deformable.q.clone();

        let v = vector![1., 1., 1.];
        let v = vec![v, v, v, v];
        let qdot0 =
            DVector::from_iterator(deformable.dof, v.iter().flat_map(|x| x.iter().copied())); // TODO: add a util fn that flattens a Vec<Vector3>
        deformable.set_velocity(v);

        // Act
        let final_time = 1.0;
        let dt = 1e-3;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            deformable.step(dt);
        }

        // Assert
        let q_expect = q0 + &qdot0 * final_time;
        assert_vec_close!(q_expect, deformable.q, 1e-3);
        assert_vec_close!(qdot0, deformable.qdot, 1e-3);
    }

    #[test]
    fn rotating() {
        // Arrange
        let mut deformable = MassSpring::unit_tetrahedron();

        let omega = vector![0.5, 2., 1.]; // angular velocity around com
        let com = deformable.com();
        let v: Vec<Vector3<Float>> = deformable
            .nodes
            .iter()
            .map(|n| omega.cross(&(n - com)))
            .collect();
        deformable.set_velocity(v);
        let angular_momentum = deformable.angular_momentum();

        // Act
        let final_time = 1.0;
        let dt = 1e-3;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            deformable.step(dt);
        }

        // Assert
        assert_vec_close!(com, deformable.com(), 1e-3);
        assert_vec_close!(
            Vector3::<Float>::zeros(),
            deformable.linear_momentum(),
            1e-3
        );
        assert_vec_close!(angular_momentum, deformable.angular_momentum(), 1e-3);
    }

    #[test]
    fn rigid_motion() {
        // Arrange
        let mut deformable = MassSpring::unit_tetrahedron();

        let v_linear = vector![1., 1., 1.]; // linear velocity
        let omega = vector![0.5, 2., 1.]; // angular velocity around com
        let com = deformable.com();
        let v: Vec<Vector3<Float>> = deformable
            .nodes
            .iter()
            .map(|n| omega.cross(&(n - com)) + v_linear)
            .collect();
        deformable.set_velocity(v);
        let linear_momentum = deformable.linear_momentum();
        let angular_momentum = deformable.angular_momentum();

        // Act
        let final_time = 1.0;
        let dt = 1e-3;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            deformable.step(dt);
        }

        // Assert
        let com_expect = com + v_linear * final_time;
        assert_vec_close!(com_expect, deformable.com(), 1e-3);
        assert_vec_close!(linear_momentum, deformable.linear_momentum(), 1e-3);
        assert_vec_close!(angular_momentum, deformable.angular_momentum(), 1e-3);
    }

    #[test]
    #[ignore = "not testing spring"]
    fn spring() {
        // Arrange
        let nodes = vec![vector![0., 0., 0.], vector![1., 0., 0.]];
        let tetrahedra = vec![vec![0, 1]];
        let mut deformable = MassSpring::new(nodes, tetrahedra);
        let q0 = deformable.q.clone();

        // let mut rng = rng();
        // let v = random_vector3(&mut rng, 1.0);
        let v = vector![0.1, 0., 0.];
        let v = vec![v, v];
        let qdot0 =
            DVector::from_iterator(deformable.dof, v.iter().flat_map(|x| x.iter().copied())); // TODO: add a util fn that flattens a Vec<Vector3>
        deformable.set_velocity(v);

        // Act
        let final_time = 5e-2;
        let dt = 1e-3;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            deformable.step(dt);
            // flog!("v: {:?}", deformable.get_velocity());
        }

        // Assert
        // assert_vec_close!(q0, deformable.q, 1e-3);
        flog!("q: {}", deformable.q);
        flog!("vel: {:#?}", deformable.get_velocity());
        assert_vec_close!(qdot0, deformable.qdot, 1e-3);
    }
}
