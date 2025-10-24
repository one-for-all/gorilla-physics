use std::collections::HashSet;

use itertools::{izip, Itertools};
use na::{dvector, vector, DMatrix, DVector, Vector3};

use crate::types::Float;

/// Mass-spring modeled two-dimensional deformable (i.e. cloth-like)
pub struct Cloth {
    pub nodes: Vec<Vector3<Float>>,
    pub faces: Vec<[usize; 3]>,

    pub edges: Vec<[usize; 2]>,

    pub q: DVector<Float>,
    pub qdot: DVector<Float>,
}

impl Cloth {
    pub fn free_velocity(&self, dt: Float) -> DVector<Float> {
        let rs: Vec<Float> = self
            .edges
            .iter()
            .map(|e| (self.nodes[e[0]] - self.nodes[e[1]]).norm())
            .collect();

        let dof = self.q.len();
        let mut f_elastic: DVector<Float> = DVector::zeros(dof);
        for (edge, r) in izip!(self.edges.iter(), rs.iter()) {
            let n0 = edge[0];
            let n1 = edge[1];
            let i_n0 = n0 * 3;
            let i_n1 = n1 * 3;
            let q0 = vector![self.q[i_n0], self.q[i_n0 + 1], self.q[i_n0 + 2]];
            let q1 = vector![self.q[i_n1], self.q[i_n1 + 1], self.q[i_n1 + 2]];

            let q0q1 = q1 - q0;
            let l = q0q1.norm();

            let k = 1e2;
            let f_n0 = k * (l - r) * q0q1 / l;
            let f_n1 = -f_n0;

            for i in 0..3 {
                f_elastic[i_n0 + i] += f_n0[i];
                f_elastic[i_n1 + i] += f_n1[i];
            }
        }

        let mut f_damping: DVector<Float> = DVector::zeros(dof);
        for edge in self.edges.iter() {
            let n0 = edge[0];
            let n1 = edge[1];
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

        let A = self.mass_matrix();
        let v0 = &self.qdot;
        let v_star = v0 - A.clone().try_inverse().unwrap() * m_v0;

        v_star
    }

    pub fn mass_matrix(&self) -> DMatrix<Float> {
        let dof = self.dof();
        let mass = 1.0;
        let M: DMatrix<Float> = mass * DMatrix::identity(dof, dof); // mass matrix
        M
    }

    pub fn dof(&self) -> usize {
        self.q.len()
    }

    fn compute_edges(faces: &Vec<[usize; 3]>) -> Vec<[usize; 2]> {
        let mut edges: HashSet<(usize, usize)> = HashSet::new();
        for face in faces.iter() {
            let i = face[0];
            let j = face[1];
            let k = face[2];
            let e1 = [i, j];
            let e2 = [j, k];
            let e3 = [k, i];
            for e_candidate in [e1, e2, e3].iter() {
                if edges.contains(&(e_candidate[0], e_candidate[1])) {
                    panic!(
                        "edge cannot be shared by two faces in its same direction: {:?}",
                        e_candidate
                    );
                }

                edges.insert((e_candidate[0], e_candidate[1]));
            }
        }

        let edges: Vec<[usize; 2]> = edges.iter().sorted().map(|e| [e.0, e.1]).collect();

        edges
    }

    /// positions of each nodes
    pub fn get_positions(&self) -> Vec<Vector3<Float>> {
        let mut p = vec![];
        let mut i = 0;
        while i < self.q.len() {
            p.push(vector![self.q[i], self.q[i + 1], self.q[i + 2]]);
            i += 3;
        }
        p
    }
}

/// Constructors
impl Cloth {
    pub fn new(nodes: Vec<Vector3<Float>>, faces: Vec<[usize; 3]>) -> Self {
        let dof = nodes.len() * 3;
        let q = DVector::from_iterator(dof, nodes.iter().flat_map(|x| x.iter().copied()));
        let qdot = DVector::zeros(q.len());

        let edges = Self::compute_edges(&faces);

        Self {
            nodes,
            faces,
            edges,
            q,
            qdot,
        }
    }

    pub fn new_square(offset: Vector3<Float>) -> Self {
        let nodes = vec![
            vector![0., 0., 0.],
            vector![1., 0., 0.],
            vector![0., 1., 0.],
            vector![1., 1., 0.],
        ]
        .iter()
        .map(|n| n + offset)
        .collect();
        let faces = vec![[0, 1, 2], [3, 2, 1]];
        Self::new(nodes, faces)
    }
}
