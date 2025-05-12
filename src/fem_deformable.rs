use itertools::izip;
use na::{vector, DVector, Matrix3, Vector3};
use nalgebra_sparse::factorization::CscCholesky;
use nalgebra_sparse::{CooMatrix, CscMatrix};
use std::collections::HashMap;
use std::ops::AddAssign;
use std::{
    fs::File,
    io::{BufReader, Read},
};

use crate::{mesh::read_mesh, types::Float};

/// Deformable modeled by finite element method
/// Ref:
///     Physics-based Animation, https://github.com/dilevin/CSC417-physics-based-animation
///     FEM Simulation of 3D Deformable Solids, 2012, Eftychios D. Sifakis
pub struct FEMDeformable {
    pub vertices: Vec<Vector3<Float>>,
    pub tetrahedra: Vec<Vec<usize>>,

    pub n_vertices: usize,

    pub B: Vec<Matrix3<Float>>, // Inverses of difference matrix of vertices of tetrahedra
    pub W: Vec<Float>, // Determinants of difference matrix of vertices of tetrahedra. i.e. 6 * volume of each tetrahedron
    pub mass_matrix: CscMatrix<Float>,
    pub mass_matrix_cholesky: CscCholesky<Float>,

    pub boundary_facets: Vec<[usize; 3]>, // TODO: consistent boundary faces outward orientation

    pub q: DVector<Float>,
    pub qdot: DVector<Float>,
}

impl FEMDeformable {
    pub fn new(vertices: Vec<Vector3<Float>>, tetrahedra: Vec<Vec<usize>>) -> Self {
        let n_vertices = vertices.len();

        let (B, W) = tetrahedra
            .iter()
            .map(|tetrahedron| {
                let vi = vertices[tetrahedron[0]];
                let vj = vertices[tetrahedron[1]];
                let vk = vertices[tetrahedron[2]];
                let vl = vertices[tetrahedron[3]];

                let D = Matrix3::<Float>::from_columns(&[vl - vi, vl - vj, vl - vk]);
                let determinant = D.determinant();
                assert!(
                    determinant > 0.0,
                    "tetrahedron {} {} {} {} has non-positive volume",
                    vi,
                    vj,
                    vk,
                    vl
                );
                let Be = D.try_inverse().expect(&format!(
                    "D should be invertible: {}, determinant: {}",
                    D, determinant
                ));
                (Be, determinant)
            })
            .unzip();

        let q = DVector::from_iterator(
            n_vertices * 3,
            vertices.iter().flat_map(|v| v.iter().copied()),
        );

        let mass_matrix = FEMDeformable::compute_mass_matrix(&tetrahedra, &W, n_vertices, 100.0);
        let mass_matrix_cholesky = CscCholesky::factor(&mass_matrix)
            .expect("Cholesky should exist because mass matrix should be positive definite");

        Self {
            vertices,
            tetrahedra,
            n_vertices,
            B,
            W,
            mass_matrix,
            mass_matrix_cholesky,
            boundary_facets: vec![],
            q,
            qdot: DVector::zeros(n_vertices * 3),
        }
    }

    /// Compute the mass matrix
    fn compute_mass_matrix(
        tetrahedra: &Vec<Vec<usize>>,
        W: &Vec<Float>,
        n_vertices: usize,
        density: Float,
    ) -> CscMatrix<Float> {
        // let mut M: CscMatrix<Float> = CscMatrix::zeros(self.n_vertices * 3, self.n_vertices * 3);
        let mut M_triplets: HashMap<(usize, usize), Float> = HashMap::new();
        for (tetrahedron, determinant) in izip!(tetrahedra.iter(), W.iter()) {
            for i in 0..4 {
                for j in 0..4 {
                    let vi = tetrahedron[i];
                    let vj = tetrahedron[j];
                    let key = (vi, vj);
                    let value = M_triplets.get(&key).unwrap_or(&0.0);
                    if vi == vj {
                        M_triplets.insert(key, value + density * determinant / 60.0);
                    // diagonal entries of mass matrix
                    } else {
                        M_triplets.insert(key, value + density * determinant / 120.0);
                        // off-diagonal entries of mass matrix
                    }
                }
            }
        }

        // Expand from n_vertices to n_vertices * 3;
        let M_triplets: Vec<(usize, usize, Float)> = M_triplets
            .iter()
            .flat_map(|(key, value)| {
                let (vi, vj) = key;
                let i = vi * 3;
                let j = vj * 3;
                let value = *value;
                vec![(i, j, value), (i + 1, j + 1, value), (i + 2, j + 2, value)]
            })
            .collect();

        let M =
            CooMatrix::try_from_triplets_iter(n_vertices * 3, n_vertices * 3, M_triplets).unwrap();
        return CscMatrix::from(&M);
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

    pub fn compute_internal_forces(&self) -> DVector<Float> {
        let mut f = DVector::zeros(self.n_vertices * 3);
        for (tetrahedron, B, W) in izip!(self.tetrahedra.iter(), self.B.iter(), self.W.iter()) {
            let i_vi = tetrahedron[0] * 3;
            let vi = vector![self.q[i_vi], self.q[i_vi + 1], self.q[i_vi + 2]];
            let i_vj = tetrahedron[1] * 3;
            let vj = vector![self.q[i_vj], self.q[i_vj + 1], self.q[i_vj + 2]];
            let i_vk = tetrahedron[2] * 3;
            let vk = vector![self.q[i_vk], self.q[i_vk + 1], self.q[i_vk + 2]];
            let i_vl = tetrahedron[3] * 3;
            let vl = vector![self.q[i_vl], self.q[i_vl + 1], self.q[i_vl + 2]];

            let D = Matrix3::<Float>::from_columns(&[vl - vi, vl - vj, vl - vk]);
            let F = D * B;

            let k = 6e3; //  Young's modulus. TODO: should be around 6e5 for rubber. reduced now for stability.
            let v = 0.4; // Poisson's ratio
            let mu = k / (2.0 * (1.0 + v));
            let lambda = k * v / ((1.0 + v) * (1.0 - 2.0 * v));
            let J = F.determinant();
            let F_inv_T = F.try_inverse().unwrap().transpose();
            let P = mu * (F - F_inv_T) + lambda * J.ln() * F_inv_T;
            let H = -W / 6.0 * P * B.transpose();

            let h1 = -H.column(0);
            let h2 = -H.column(1);
            let h3 = -H.column(2);

            f.rows_mut(i_vi, 3).add_assign(h1);
            f.rows_mut(i_vj, 3).add_assign(h2);
            f.rows_mut(i_vk, 3).add_assign(h3);
            f.rows_mut(i_vl, 3).add_assign(-h1 - h2 - h3);
        }
        f
    }

    pub fn step(&mut self, dt: Float, tau: &DVector<Float>) {
        let tau: &DVector<Float> = {
            if tau.len() != 0 {
                tau
            } else {
                &DVector::zeros(self.n_vertices * 3)
            }
        };

        let internal_force = self.compute_internal_forces();
        let qddot = self.mass_matrix_cholesky.solve(&(internal_force + tau));
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

pub fn read_fem_box() -> FEMDeformable {
    let path = "data/box.mesh";
    let file = File::open(path).expect(&format!("{} should exist", path));
    let mut reader = BufReader::new(file);

    let mut buf: String = String::new();
    let _ = reader.read_to_string(&mut buf);

    let (vertices, tetrahedra) = read_mesh(&buf);
    FEMDeformable::new(vertices, tetrahedra)
}

#[cfg(test)]
mod fem_deformable_tests {

    use na::dvector;

    use crate::assert_vec_close;

    use super::*;

    /// Stays stationay under no force
    #[test]
    fn stationary() {
        // Arrange
        let mut deformable = read_fem_box();
        let initial_q = deformable.q.clone();
        let initial_qdot = deformable.qdot.clone();

        // Act
        let final_time = 5.0;
        let dt = 1.0 / 600.0;
        let num_steps = (final_time / dt) as usize;
        for _ in 0..num_steps {
            deformable.step(dt, &dvector![]);
        }

        // Assert
        assert_vec_close!(initial_q, deformable.q, 1e-2);
        assert_vec_close!(initial_qdot, deformable.qdot, 1e-2);
    }

    /// Constant velocity
    #[test]
    fn constant_velocity() {
        // Arrange
        let mut deformable = read_fem_box();
        let initial_qdot = DVector::from_element(deformable.n_vertices * 3, 1.0);
        deformable.qdot = initial_qdot.clone();

        // Act
        let final_time = 3.0;
        let dt = 1.0 / 600.0;
        let num_steps = (final_time / dt) as usize;
        for _ in 0..num_steps {
            deformable.step(dt, &dvector![]);
        }

        // Assert
        assert_vec_close!(initial_qdot, deformable.qdot, 1e-2);
    }

    /// Drag the body by a point, and expect whole body translation
    #[test]
    fn drag_at_a_point() {
        // Arrange
        let mut deformable = read_fem_box();
        let initial_q = deformable.q.clone();

        // Act
        let direction = vector![1.0, 1.0, 1.0];
        let action_point_index = deformable.extreme_point(&direction);
        let mut tau = DVector::zeros(deformable.n_vertices * 3);
        let force = 1e2;
        tau[action_point_index * 3] = force;
        tau[action_point_index * 3 + 1] = force;
        tau[action_point_index * 3 + 2] = force;

        let final_time = 2.0;
        let dt = 1.0 / 600.0;
        let num_steps = (final_time / dt) as usize;
        for _ in 0..num_steps {
            deformable.step(dt, &tau);
        }

        // Assert
        let end_q = deformable.q;
        let diff_q = end_q - initial_q;
        for i in 0..deformable.n_vertices {
            let v_diff_q = vector![diff_q[i * 3], diff_q[i * 3 + 1], diff_q[i * 3 + 2]];
            assert!(
                v_diff_q.dot(&direction) > 0.0,
                "{}th point did not move towards the drag direction, diff: {}",
                i + 1,
                v_diff_q
            );
        }
    }
}
