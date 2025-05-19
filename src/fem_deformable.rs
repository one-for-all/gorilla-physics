use clarabel::algebra::VectorMath;
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
    pub mass_matrix_lumped: DVector<Float>, // Masses lumped to the vertices, i.e. all the masses of the deformable are assumed to be on the vertices

    pub density: Float,
    pub mu: Float,     // Lamé coefficients, μ
    pub lambda: Float, // Lamé coefficients, λ

    pub boundary_facets: Vec<[usize; 3]>, // TODO: consistent boundary faces outward orientation

    pub q: DVector<Float>,
    pub qdot: DVector<Float>,
}

impl FEMDeformable {
    /// Create a FEM-based deformable
    /// k is Young's modulus, i.e. a measure of stretch resistance
    /// v is Poisson's ratio, i.e. a measure of incompressibility
    pub fn new(
        vertices: Vec<Vector3<Float>>,
        tetrahedra: Vec<Vec<usize>>,
        density: Float,
        k: Float,
        v: Float,
    ) -> Self {
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

        let mass_matrix = FEMDeformable::compute_mass_matrix(&tetrahedra, &W, n_vertices, density);
        let mass_matrix_cholesky = CscCholesky::factor(&mass_matrix)
            .expect("Cholesky should exist because mass matrix should be positive definite");

        let mass_matrix_lumped: DVector<Float> = DVector::from_vec(
            mass_matrix
                .col_iter()
                .map(|x| x.values().sum())
                .collect::<Vec<_>>(),
        );

        // Compute Lamé coefficients from Young's modulus and Poisson's ratio
        // Ref:
        //      The classical FEM method and discretization methodology,
        //      Eftychios D. Sifakis, 2012, Section 3.2 Linear Elasticity
        let mu = k / (2.0 * (1.0 + v));
        let lambda = k * v / ((1.0 + v) * (1.0 - 2.0 * v));

        Self {
            vertices,
            tetrahedra,
            n_vertices,
            B,
            W,
            mass_matrix,
            mass_matrix_cholesky,
            mass_matrix_lumped,
            density,
            mu,
            lambda,
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

    pub fn compute_internal_forces(&self, q: &DVector<Float>) -> DVector<Float> {
        let mut f = DVector::zeros(self.n_vertices * 3);
        for (tetrahedron, B, W) in izip!(self.tetrahedra.iter(), self.B.iter(), self.W.iter()) {
            let i_vi = tetrahedron[0] * 3;
            let i_vj = tetrahedron[1] * 3;
            let i_vk = tetrahedron[2] * 3;
            let i_vl = tetrahedron[3] * 3;

            let F = FEMDeformable::compute_deformable_gradients(q, tetrahedron, B);

            let J = F.determinant();
            let F_inv_T = F.try_inverse().unwrap().transpose();
            let P = self.mu * (F - F_inv_T) + self.lambda * J.ln() * F_inv_T;
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
        let tau: DVector<Float> = {
            if tau.len() != 0 {
                tau.clone()
            } else {
                DVector::zeros(self.n_vertices * 3)
            }
        };

        // // TODO: Keep here for hack testing. Remove it later.
        // // It simulates gravity on the deformable.
        // for (tetrahedron, determinant) in izip!(self.tetrahedra.iter(), self.W.iter()) {
        //     let volume = determinant;
        //     let v0 = tetrahedron[0];
        //     let v1 = tetrahedron[1];
        //     let v2 = tetrahedron[2];
        //     let v3 = tetrahedron[3];
        //     let gravity_force = -9.81 * self.density * volume / 4.0;
        //     tau[v0 * 3 + 2] += gravity_force;
        //     tau[v1 * 3 + 2] += gravity_force;
        //     tau[v2 * 3 + 2] += gravity_force;
        //     tau[v3 * 3 + 2] += gravity_force;
        // }

        // // TODO: Keep here for hack testing. Remove it later.
        // // It simulates a half-space for collision with the body.
        // let plane = -1.0;
        // for i in 0..self.n_vertices {
        //     let z = self.q[i * 3 + 2];
        //     if z < plane {
        //         let depth = plane - z;
        //         let zn = depth.powf(3.0 / 2.0);
        //         let vz = self.qdot[i * 3 + 2];
        //         let z_dot = -vz;
        //         let k = 50e2;
        //         let a = 1.0;
        //         let λ = 3.0 / 2.0 * a * k;
        //         let π = (λ * zn * z_dot + k * zn).max(0.0);
        //         tau[i * 3 + 2] += π;
        //     }
        // }

        // TODO: This procedure is extremely slow. Need to speed it up.
        let mut i = 0;
        let max_iteration = 100;
        let mut delta_x = DVector::repeat(self.n_vertices * 3, Float::INFINITY);
        let mut q_next = self.q.clone(); // set initial guesses of q and qdot
        let mut qdot_next = self.qdot.clone();
        while i < max_iteration && !(delta_x.abs().max() < 1e-6) {
            // Solve the linearized equation at this q to get better q, qdot estimate
            let f = self.compute_internal_forces(&q_next) + &tau;
            let b = (&self.qdot - &qdot_next).component_mul(&self.mass_matrix_lumped) / dt + f;

            let Fs: Vec<Matrix3<Float>> = izip!(self.tetrahedra.iter(), self.B.iter())
                .map(|(tetrahedron, B)| {
                    FEMDeformable::compute_deformable_gradients(&q_next, tetrahedron, B)
                })
                .collect();
            let (Js, F_invs): (Vec<Float>, Vec<Matrix3<Float>>) = Fs
                .iter()
                .map(|F| (F.determinant(), F.try_inverse().unwrap()))
                .collect::<(Vec<Float>, Vec<Matrix3<Float>>)>();
            let F_inv_Ts: Vec<Matrix3<Float>> =
                F_invs.iter().map(|F_inv| F_inv.transpose()).collect();
            delta_x = conjugate_gradient(
                |x| {
                    self.compute_force_differential(&-x, &Fs, &Js, &F_invs, &F_inv_Ts)
                        + self.mass_matrix_lumped.component_mul(&x) / (dt * dt)
                },
                &b,
                &q_next,
                1e-3,
            );
            q_next += &delta_x;
            qdot_next += &delta_x / dt;
            i += 1;
        }
        self.qdot = qdot_next;
        self.q += dt * &self.qdot;

        // let internal_force = self.compute_internal_forces(&self.q);
        // // let qddot = self.mass_matrix_cholesky.solve(&(internal_force + tau));
        // let qddot = (internal_force + tau).component_div(&self.mass_matrix_lumped);
        // self.qdot += dt * qddot;
        // self.q += dt * &self.qdot;
    }

    /// Computes the dF for a given dq at current q.
    /// i.e. df = -K dq, where K is the stiffness matrix, aka d^2V/dq^2
    /// Fs is a vec of deformable gradients, one for each tetrahedron
    /// Ref:
    ///     FEM Simulation of 3D Deformable Solids: A practitioner’s guide to theory, discretization and model reduction.
    ///     Part One: The classical FEM method and discretization methodology,
    ///     Eftychios D. Sifakis, 2012, Section 4.3 Force differentials
    fn compute_force_differential(
        &self,
        dq: &DVector<Float>,
        Fs: &Vec<Matrix3<Float>>,
        Js: &Vec<Float>,
        F_invs: &Vec<Matrix3<Float>>,
        F_inv_Ts: &Vec<Matrix3<Float>>,
    ) -> DVector<Float> {
        let mut df = DVector::zeros(self.n_vertices * 3);
        for (tetrahedron, B, W, F, J, F_inv, F_inv_T) in izip!(
            self.tetrahedra.iter(),
            self.B.iter(),
            self.W.iter(),
            Fs.iter(),
            Js.iter(),
            F_invs.iter(),
            F_inv_Ts.iter()
        ) {
            let i_vi = tetrahedron[0] * 3;
            let i_vj = tetrahedron[1] * 3;
            let i_vk = tetrahedron[2] * 3;
            let i_vl = tetrahedron[3] * 3;

            let dvi = vector![dq[i_vi], dq[i_vi + 1], dq[i_vi + 2]];
            let dvj = vector![dq[i_vj], dq[i_vj + 1], dq[i_vj + 2]];
            let dvk = vector![dq[i_vk], dq[i_vk + 1], dq[i_vk + 2]];
            let dvl = vector![dq[i_vl], dq[i_vl + 1], dq[i_vl + 2]];
            let dD = Matrix3::<Float>::from_columns(&[dvl - dvi, dvl - dvj, dvl - dvk]);
            let dF = dD * B;

            let F_inv_dF = F_inv * dF;
            let dP = self.mu * dF
                + (self.mu - self.lambda * J.ln()) * F_inv_T * F_inv_dF.transpose()
                + self.lambda * F_inv_dF.trace() * F_inv_T;
            let dH = -W / 6.0 * dP * B.transpose();

            let dh1 = -dH.column(0);
            let dh2 = -dH.column(1);
            let dh3 = -dH.column(2);

            df.rows_mut(i_vi, 3).add_assign(dh1);
            df.rows_mut(i_vj, 3).add_assign(dh2);
            df.rows_mut(i_vk, 3).add_assign(dh3);
            df.rows_mut(i_vl, 3).add_assign(-dh1 - dh2 - dh3);
        }
        df
    }

    /// Computes the deformable gradient for a single tetrahedron
    /// i.e. returns F = dV/dq
    fn compute_deformable_gradients(
        q: &DVector<Float>,
        tetrahedron: &Vec<usize>,
        B: &Matrix3<Float>,
    ) -> Matrix3<Float> {
        let i_vi = tetrahedron[0] * 3;
        let i_vj = tetrahedron[1] * 3;
        let i_vk = tetrahedron[2] * 3;
        let i_vl = tetrahedron[3] * 3;
        let vi = vector![q[i_vi], q[i_vi + 1], q[i_vi + 2]];
        let vj = vector![q[i_vj], q[i_vj + 1], q[i_vj + 2]];
        let vk = vector![q[i_vk], q[i_vk + 1], q[i_vk + 2]];
        let vl = vector![q[i_vl], q[i_vl + 1], q[i_vl + 2]];
        let D = Matrix3::<Float>::from_columns(&[vl - vi, vl - vj, vl - vk]);
        D * B
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

/// Conjugate Gradient method for solving linear system A x = b, where A is
/// positive-definite.
/// Ref:
///     An Introduction to the Conjugate Gradient Method Without the Agonizing
///     Pain, Jonathan Richard Shewchuk, 1994, Section B2. Conjugate Gradients
fn conjugate_gradient<F>(
    A_mul: F,
    b: &DVector<Float>,
    x0: &DVector<Float>,
    tol: Float,
) -> DVector<Float>
where
    F: Fn(&DVector<Float>) -> DVector<Float>,
{
    let mut x = x0.clone();

    let mut i = 1;
    let mut r = b - A_mul(x0);
    let mut d = r.clone();
    let mut delta_new = r.norm_squared();

    let max_iteration = 501;
    // TODO: guard against or warn on NaN?
    while i < max_iteration && !(delta_new < tol) {
        let q = &A_mul(&d);
        let alpha = delta_new / d.dot(&q);
        x += alpha * &d;
        if i % 50 == 0 {
            r = b - A_mul(&x);
        } else {
            r -= alpha * q;
        }
        let delta_old = delta_new;
        delta_new = r.norm_squared();
        let beta = delta_new / delta_old;
        d = &r + beta * &d;
        i += 1;
    }
    x
}

pub fn read_fem_box() -> FEMDeformable {
    let path = "data/box.mesh";
    let file = File::open(path).expect(&format!("{} should exist", path));
    let mut reader = BufReader::new(file);

    let mut buf: String = String::new();
    let _ = reader.read_to_string(&mut buf);

    let (vertices, tetrahedra) = read_mesh(&buf);
    FEMDeformable::new(vertices, tetrahedra, 100.0, 6e5, 0.4)
}

#[cfg(test)]
mod solver_tests {
    use na::{Cholesky, DMatrix, DVector};
    use rand::Rng;

    use crate::{assert_vec_close, types::Float};

    use super::conjugate_gradient;

    #[test]
    fn conjugate_gradient_random() {
        // Arrange
        let mut rng = rand::rng();

        for _ in 0..100 {
            let n = rng.random_range(1..=100);
            let x0 = DVector::zeros(n);

            // Create a random matrix A of size (n x n)
            let a_data: Vec<Float> = (0..n * n).map(|_| rng.random_range(-1.0..1.0)).collect();
            let a = DMatrix::from_vec(n, n, a_data);

            // Compute A * A^T to get a symmetric positive semi-definite matrix
            let mut A = &a * a.transpose();

            // Ad n * I to make it strictly positive-definite
            A += DMatrix::<Float>::identity(n, n) * (n as Float);

            let b: DVector<Float> =
                DVector::from_vec((0..n).map(|_| rng.random_range(-1.0..1.0)).collect());

            // Act
            let x_cg = conjugate_gradient(|x| &A * x, &b, &x0, 1e-10);

            // Assert
            let x_cholesky = Cholesky::new(A).unwrap().solve(&b);
            assert_vec_close!(x_cg, x_cholesky, 1e-5);
        }
    }
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
        let dt = 1.0 / 60.0;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
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
        let dt = 1.0 / 60.0;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
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
