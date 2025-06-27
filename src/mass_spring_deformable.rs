use std::{
    fs::File,
    io::{BufReader, Read},
};

use itertools::izip;
use na::{vector, DVector, Vector3};
use nalgebra_sparse::{CooMatrix, CscMatrix, CsrMatrix};

use crate::{mesh::read_mesh, types::Float, util::read_file};

/// Deformable modeled as mass-spring system.
/// Ref: Physics-based Animation, https://github.com/dilevin/CSC417-physics-based-animation
pub struct MassSpringDeformable {
    pub vertices: Vec<Vector3<Float>>,
    pub tetrahedra: Vec<Vec<usize>>,

    pub m: Float, // total mass
    pub k: Float, // spring constant of each edge/spring

    pub n_vertices: usize,
    pub edges: Vec<(usize, usize)>,
    pub boundary_facets: Vec<[usize; 3]>, // TODO: consistent boundary faces outward orientation

    pub q: DVector<Float>, // generalized coordinates of the mesh, i.e. the vertex coordinates
    pub qdot: DVector<Float>,

    rest_spring_lengths: Vec<Float>, // rest lengths of the edges/springs
    mass_matrix: CscMatrix<Float>, // Unnecessary for mass-spring system, since it is just a diagonable matrix w/ identical elements
}

impl MassSpringDeformable {
    /// Builds up a deformable modeled as a mass-spring system.
    /// m is the total mass of the deformable
    pub fn new(
        vertices: Vec<Vector3<Float>>,
        tetrahedra: Vec<Vec<usize>>,
        m: Float,
        k: Float,
    ) -> Self {
        let n_vertices = vertices.len();

        let q = DVector::from_iterator(
            n_vertices * 3,
            vertices.iter().flat_map(|v| v.iter().copied()),
        );

        let mut deformable = Self {
            vertices,
            tetrahedra,
            m,
            k,
            n_vertices,
            edges: vec![],
            boundary_facets: vec![],
            q,
            qdot: DVector::zeros(n_vertices * 3),
            rest_spring_lengths: vec![],
            mass_matrix: CscMatrix::zeros(0, 0),
        };

        deformable.extract_edges();
        deformable.compute_mass_matrix(m / n_vertices as Float);
        deformable
    }

    /// Extract all the unique edges and store in self
    fn extract_edges(&mut self) {
        self.edges = vec![];
        self.rest_spring_lengths = vec![];

        let n_dim = self.n_vertices;

        // Build adjacency matrix
        let mut adj: CooMatrix<u8> = CooMatrix::new(n_dim, n_dim);
        for tetrahedron in self.tetrahedra.iter() {
            for i in 0..3 {
                for j in (i + 1)..4 {
                    let v0 = tetrahedron[i];
                    let v1 = tetrahedron[j];
                    adj.push(v0, v1, 1);
                    adj.push(v1, v0, 1);
                }
            }
        }
        let adj = CsrMatrix::from(&adj);

        for irow in 0..n_dim {
            let row = adj.row(irow);
            for icol in row.col_indices() {
                if irow < *icol {
                    self.edges.push((irow, *icol));
                    let v0 = self.vertices[irow];
                    let v1 = self.vertices[*icol];
                    self.rest_spring_lengths.push((v0 - v1).norm());
                }
            }
        }
    }

    /// Extract the boundary facets, for better visualization.
    pub fn extract_boundary_facets(&mut self) {
        let mut facets = vec![];
        // Collect all the faces in the tetrahedra
        for tetrahedron in &self.tetrahedra {
            let v0 = tetrahedron[0];
            let v1 = tetrahedron[1];
            let v2 = tetrahedron[2];
            let v3 = tetrahedron[3];

            facets.push({
                let mut v = [v0, v1, v2];
                v.sort();
                v
            });
            facets.push({
                let mut v = [v0, v1, v3];
                v.sort();
                v
            });
            facets.push({
                let mut v = [v0, v2, v3];
                v.sort();
                v
            });
            facets.push({
                let mut v = [v1, v2, v3];
                v.sort();
                v
            });
        }
        facets.sort();

        // Get the faces that only appear once. They are the boundary faces.
        let mut boundary_facets = vec![];
        for i in 0..facets.len() {
            let cur = facets[i];
            if i > 0 && cur == facets[i - 1] {
                continue;
            }
            if i < facets.len() - 1 && cur == facets[i + 1] {
                continue;
            }
            boundary_facets.push(cur);
        }

        self.boundary_facets = boundary_facets;
    }

    /// Returns the mass matrix of the mesh, where each point has mass m
    fn compute_mass_matrix(&mut self, m: Float) {
        self.mass_matrix = CscMatrix::identity(self.n_vertices * 3) * m
    }

    /// Computes the gradient of the potential energy V to the generalized
    /// coordinates q, i.e. the negative of the generalized force
    pub fn dV_dq(&self) -> DVector<Float> {
        let mut result = DVector::zeros(self.n_vertices * 3);

        for (edge, rest_l) in izip!(self.edges.iter(), self.rest_spring_lengths.iter()) {
            let row_v0 = edge.0;
            let row_v1 = edge.1;
            let i_v0 = row_v0 * 3;
            let i_v1 = row_v1 * 3;
            let q0 = vector![self.q[i_v0], self.q[i_v0 + 1], self.q[i_v0 + 2]];
            let q1 = vector![self.q[i_v1], self.q[i_v1 + 1], self.q[i_v1 + 2]];

            let q0_minus_q1 = q0 - q1;
            let norm = q0_minus_q1.norm();

            // the update for q0 entry, and also is the negative of the udpate to q1 entry
            let dV_dq_element = self.k / norm * (norm - rest_l) * q0_minus_q1;

            for i in 0..3 {
                result[i_v0 + i] += dV_dq_element[i];
                result[i_v1 + i] -= dV_dq_element[i];
            }
        }

        result
    }

    pub fn step(&mut self, dt: Float, tau: &DVector<Float>) {
        let tau: &DVector<Float> = {
            if tau.len() != 0 {
                tau
            } else {
                &DVector::zeros(self.n_vertices * 3)
            }
        };

        // TODO: Do implicit integration for better stability.
        let qddot = (-self.dV_dq() + tau) / (self.m / self.n_vertices as Float);
        self.qdot += dt * qddot;
        self.q += dt * &self.qdot;
    }

    /// Returns the index of the point that is furthest in given direction
    pub fn extreme_point(&self, direction: &Vector3<Float>) -> usize {
        let mut index = 0;
        let mut max_distance = 0.0;
        for i in 0..self.n_vertices {
            let i_v = i * 3;
            let q = vector![self.q[i_v], self.q[i_v + 1], self.q[i_v + 2]];
            let distance = q.dot(direction);
            if distance > max_distance {
                index = i;
                max_distance = distance;
            }
        }
        index
    }
}

pub fn read_mass_spring_bunny() -> MassSpringDeformable {
    let file_path = "data/coarse_bunny.mesh";
    let buf = read_file(file_path);

    let (vertices, tetrahedra) = read_mesh(&buf);
    let n_vertices = vertices.len() as Float;
    MassSpringDeformable::new(vertices, tetrahedra, 100.0 * n_vertices, 1e5)
}

#[cfg(test)]
mod mass_spring_deformable_tests {
    use na::dvector;

    use crate::assert_vec_close;

    use super::*;

    /// Stays stationay under no force
    #[test]
    fn stationary() {
        // Arrange
        let mut bunny = read_mass_spring_bunny();
        let initial_q = bunny.q.clone();
        let initial_qdot = bunny.qdot.clone();

        // Act
        let dt = 1e-2;
        for _ in 0..10 {
            bunny.step(dt, &dvector![]);
        }

        // Assert
        assert_eq!(initial_q, bunny.q);
        assert_eq!(initial_qdot, bunny.qdot);
    }

    /// Constant velocity
    #[test]
    fn constant_velocity() {
        // Arrange
        let mut bunny = read_mass_spring_bunny();
        let initial_qdot = DVector::from_element(bunny.n_vertices * 3, 1.0);
        bunny.qdot = initial_qdot.clone();

        // Act
        let final_time = 3.0;
        let dt = 1e-2;
        let num_steps = (final_time / dt) as usize;
        for _ in 0..num_steps {
            bunny.step(dt, &dvector![]);
        }

        // Assert
        assert_vec_close!(initial_qdot, bunny.qdot, 1e-2);
    }

    /// Drag the body by a point, and expect whole body translation
    #[test]
    fn drag_at_a_point() {
        // Arrange
        let mut bunny = read_mass_spring_bunny();
        let initial_q = bunny.q.clone();

        // Act
        let action_point_index = bunny.extreme_point(&vector![1.0, 0.0, 0.0]);
        let mut tau = DVector::zeros(bunny.n_vertices * 3);
        tau[action_point_index * 3] = 1e4;

        let final_time = 2.0;
        let dt = 1e-3;
        let num_steps = (final_time / dt) as usize;
        for _ in 0..num_steps {
            bunny.step(dt, &tau);
        }

        // Assert
        let end_q = bunny.q;
        let diff_q = end_q - initial_q;
        for i in 0..bunny.n_vertices {
            assert!(
                diff_q[i * 3] > 0.0,
                "{}th point did not move towards the drag direction, diff: {}",
                i + 1,
                diff_q[i * 3]
            );
        }
    }
}
