use clarabel::algebra::CscMatrix as ClarabelCscMatrix;
use clarabel::algebra::VectorMath;
use itertools::izip;
use na::UnitVector3;
use na::{vector, DVector, Matrix3, Vector3};
use nalgebra_sparse::{CooMatrix, CscMatrix};
use std::collections::HashMap;
use std::future::Future;
use std::ops::AddAssign;

use crate::collision::mesh::read_mesh;
use crate::contact::HalfSpace;
use crate::dynamics::solve_cone_complementarity;
use crate::gpu::{
    async_initialize_gpu, compute_dH, setup_compute_dH_pipeline, Matrix3x3, WgpuContext,
};
use crate::types::Float;
use crate::util::read_file;
use crate::GRAVITY;

/// Deformable modeled by finite element method
/// Ref:
///     Physics-based Animation, https://github.com/dilevin/CSC417-physics-based-animation
///     FEM Simulation of 3D Deformable Solids, 2012, Eftychios D. Sifakis
pub struct FEMDeformable {
    pub vertices: Vec<Vector3<Float>>, // Only set at construction. Not updated by stepping.
    pub tetrahedra: Vec<Vec<usize>>,

    pub n_vertices: usize,

    pub Bs: Vec<Matrix3<Float>>, // Inverses of difference matrix of vertices of tetrahedra
    pub Ws: Vec<Float>, // (Determinants divided by 6) of difference matrix of vertices of tetrahedra. i.e. undeformed volume of each tetrahedron. Volumne = determminant / 6
    pub mass_matrix: CscMatrix<Float>,
    // mass_matrix_cholesky: CscCholesky<Float>,
    pub mass_matrix_lumped: DVector<Float>, // Masses lumped to the vertices, i.e. all the masses of the deformable are assumed to be on the vertices

    pub density: Float,
    pub mu: Float,     // Lamé coefficients, μ
    pub lambda: Float, // Lamé coefficients, λ

    pub boundary_facets: Vec<[usize; 3]>, // TODO: consistent boundary faces outward orientation

    pub q: DVector<Float>,
    pub qdot: DVector<Float>,

    // for working with GPU
    pub wgpu_context: WgpuContext,

    halfspaces: Vec<HalfSpace>, // TODO: store halfspaces in a container, not inside FEMDeformable
    pub enable_gravity: bool,   // TODO: remove this when FEM is integrated w/ rigid body
}

impl FEMDeformable {
    /// Create a FEM-based deformable
    /// k is Young's modulus, a measure of stretch resistance
    /// v is Poisson's ratio, a measure of incompressibility
    pub async fn new(
        vertices: Vec<Vector3<Float>>,
        tetrahedra: Vec<Vec<usize>>,
        density: Float,
        k: Float,
        v: Float,
    ) -> Self {
        let n_vertices = vertices.len();

        let (Bs, Ws): (Vec<Matrix3<Float>>, Vec<Float>) = tetrahedra
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
                (Be, determinant / 6.0)
            })
            .unzip();

        let q = DVector::from_iterator(
            n_vertices * 3,
            vertices.iter().flat_map(|v| v.iter().copied()),
        );

        let mass_matrix = FEMDeformable::compute_mass_matrix(&tetrahedra, &Ws, n_vertices, density);
        // let mass_matrix_cholesky = CscCholesky::factor(&mass_matrix)
        //     .expect("Cholesky should exist because mass matrix should be positive definite");

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

        // Initialize GPU
        let (device, queue) = async_initialize_gpu().await;

        let padded_Bs: &Vec<Matrix3x3> = &(Bs
            .iter()
            .map(|mat| {
                let mut cols = [[0.0; 4]; 3];
                for i in 0..3 {
                    cols[i][0] = mat[(0, i)] as f32;
                    cols[i][1] = mat[(1, i)] as f32;
                    cols[i][2] = mat[(2, i)] as f32;
                }
                Matrix3x3 { cols }
            })
            .collect());

        let (
            shader_module,
            pipeline,
            bind_group,
            input_dq_buffer,
            input_F_invs_buffer,
            input_Js_buffer,
            output_dH_buffer,
            download_dH_buffer,
        ) = setup_compute_dH_pipeline(
            &device,
            n_vertices,
            &tetrahedra,
            padded_Bs,
            &Ws.iter().map(|x| *x as f32).collect(),
            mu as f32,
            lambda as f32,
        );

        let wgpu_context = WgpuContext {
            device,
            queue,
            module: shader_module,
            pipeline,
            bind_group,
            input_buffers: HashMap::from([
                ("dq", input_dq_buffer),
                ("F_invs", input_F_invs_buffer),
                ("Js", input_Js_buffer),
            ]),
            output_buffers: HashMap::from([("dH", output_dH_buffer)]),
            download_buffers: HashMap::from([("dH", download_dH_buffer)]),
        };
        Self {
            vertices,
            tetrahedra,
            n_vertices,
            Bs,
            Ws,
            mass_matrix,
            // mass_matrix_cholesky,
            mass_matrix_lumped,
            density,
            mu,
            lambda,
            boundary_facets: vec![],
            q,
            qdot: DVector::zeros(n_vertices * 3),
            wgpu_context,
            halfspaces: vec![],
            enable_gravity: false,
        }
    }

    pub fn add_halfspace(&mut self, halfspace: HalfSpace) {
        self.halfspaces.push(halfspace);
    }

    /// Compute the mass matrix
    fn compute_mass_matrix(
        tetrahedra: &Vec<Vec<usize>>,
        W: &Vec<Float>,
        n_vertices: usize,
        density: Float,
    ) -> CscMatrix<Float> {
        let mut M_triplets: HashMap<(usize, usize), Float> = HashMap::new();
        for (tetrahedron, volume) in izip!(tetrahedra.iter(), W.iter()) {
            for i in 0..4 {
                for j in 0..4 {
                    let vi = tetrahedron[i];
                    let vj = tetrahedron[j];
                    let key = (vi, vj);
                    let value = M_triplets.get(&key).unwrap_or(&0.0);
                    if vi == vj {
                        // diagonal entries of mass matrix
                        M_triplets.insert(key, value + density * volume / 10.0);
                    } else {
                        // off-diagonal entries of mass matrix
                        M_triplets.insert(key, value + density * volume / 20.0);
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

    /// Compute internal elastic force f = -dV/dq, where V is the elastic energy
    pub fn compute_internal_forces(&self, q: &DVector<Float>) -> DVector<Float> {
        let mut f = DVector::zeros(self.n_vertices * 3);
        for (tetrahedron, B, volume) in
            izip!(self.tetrahedra.iter(), self.Bs.iter(), self.Ws.iter())
        {
            let i_vi = tetrahedron[0] * 3;
            let i_vj = tetrahedron[1] * 3;
            let i_vk = tetrahedron[2] * 3;
            let i_vl = tetrahedron[3] * 3;

            let F = FEMDeformable::compute_deformation_gradients(q, tetrahedron, B);

            let J = F.determinant();
            let F_inv_T = F.try_inverse().unwrap().transpose();
            let P = self.mu * (F - F_inv_T) + self.lambda * J.ln() * F_inv_T;
            let H = *volume * P * B.transpose();

            let h1 = H.column(0);
            let h2 = H.column(1);
            let h3 = H.column(2);

            f.rows_mut(i_vi, 3).add_assign(h1);
            f.rows_mut(i_vj, 3).add_assign(h2);
            f.rows_mut(i_vk, 3).add_assign(h3);
            f.rows_mut(i_vl, 3).add_assign(-h1 - h2 - h3);
        }
        f
    }

    /// Compute E = 1/2 * (qdot_new - qdot)^T*M*(qdot_new - qdot) + V(q +
    /// qdot_new * dt)
    /// Note: This is not the energy of the system, but an energy to be
    /// minimized for velocity stepping
    /// Ref:
    ///  1. Physics-based animation lecture 5: OH NO! It's More Finite Elements
    ///     https://www.youtube.com/watch?v=RsdyeUyWss0&ab_channel=DavidLevin
    ///  2. The classical FEM method and discretization methodology,
    ///     Eftychios D. Sifakis, 2012, Section 3.6 Neohookean elasticity
    pub fn compute_energy(&self, qdot_new: &DVector<Float>, dt: Float) -> Float {
        let qdot_diff = qdot_new - &self.qdot;
        let q_new = &self.q + qdot_new * dt;

        let momentum_potential =
            0.5 * (&qdot_diff.component_mul(&self.mass_matrix_lumped)).dot(&qdot_diff);
        let gravitational_energy: Float = izip!(self.tetrahedra.iter(), self.Ws.iter())
            .map(|(tetrahedron, volume)| {
                let i_z0 = tetrahedron[0] * 3 + 2;
                let i_z1 = tetrahedron[1] * 3 + 2;
                let i_z2 = tetrahedron[2] * 3 + 2;
                let i_z3 = tetrahedron[3] * 3 + 2;
                let mass = self.density * volume / 4.0;
                mass * GRAVITY * (q_new[i_z0] + q_new[i_z1] + q_new[i_z2] + q_new[i_z3])
            })
            .sum();
        let elastic_energy: Float = izip!(self.tetrahedra.iter(), self.Ws.iter(), self.Bs.iter())
            .map(|(tetrahedron, volume, B)| {
                let F = FEMDeformable::compute_deformation_gradients(&q_new, tetrahedron, B);
                let I1 = (F.transpose() * F).trace();
                let J_ln = F.determinant().ln();

                let psi =
                    self.mu / 2.0 * (I1 - 3.0) - self.mu * J_ln + self.lambda / 2.0 * J_ln * J_ln;
                psi * volume
            })
            .sum();
        momentum_potential + gravitational_energy + elastic_energy
    }

    pub async fn step(&mut self, dt: Float, tau: &DVector<Float>) {
        let mut tau: DVector<Float> = {
            if tau.len() != 0 {
                tau.clone()
            } else {
                DVector::zeros(self.n_vertices * 3)
            }
        };

        // TODO: Keep here for hack testing. Remove it later.
        // It simulates gravity on the deformable.
        if self.enable_gravity {
            for (tetrahedron, volume) in izip!(self.tetrahedra.iter(), self.Ws.iter()) {
                let volume = volume;
                let v0 = tetrahedron[0];
                let v1 = tetrahedron[1];
                let v2 = tetrahedron[2];
                let v3 = tetrahedron[3];
                let gravity_force = -GRAVITY * self.density * volume / 4.0;
                tau[v0 * 3 + 2] += gravity_force;
                tau[v1 * 3 + 2] += gravity_force;
                tau[v2 * 3 + 2] += gravity_force;
                tau[v3 * 3 + 2] += gravity_force;
            }
        }

        // TODO: This procedure can be slow. Find ways to speed it up.
        let mut i = 0;
        let max_iteration = 10;
        let mut delta_x = DVector::repeat(self.n_vertices * 3, Float::INFINITY);
        let mut q_new = self.q.clone(); // set initial guesses of q and qdot
        let mut qdot_new = self.qdot.clone();
        let dt_squared = dt * dt;
        while i < max_iteration && !(delta_x.abs().max() < 1e-6) {
            // Solve the linearized equation at this q to get better q, qdot estimates
            let f = self.compute_internal_forces(&q_new) + &tau;
            let b = (&self.qdot - &qdot_new).component_mul(&self.mass_matrix_lumped) / dt + f;

            let Fs: Vec<Matrix3<Float>> = izip!(self.tetrahedra.iter(), self.Bs.iter())
                .map(|(tetrahedron, B)| {
                    FEMDeformable::compute_deformation_gradients(&q_new, tetrahedron, B)
                })
                .collect();
            let (Js, F_invs): (Vec<Float>, Vec<Matrix3<Float>>) = Fs
                .iter()
                .map(|F| (F.determinant(), F.try_inverse().unwrap()))
                .collect::<(Vec<Float>, Vec<Matrix3<Float>>)>();

            // initial guess of delta x
            let dx0: DVector<Float> = DVector::repeat(self.n_vertices * 3, 0.0);

            let mass_matrix_lumped = &self.mass_matrix_lumped;
            let n_vertices = self.n_vertices;
            let tetrahedra = &self.tetrahedra;
            if cfg!(feature = "gpu") {
                let wgpu_context = &self.wgpu_context;
                let Js_f32: Vec<f32> = Js.iter().map(|x| *x as f32).collect();
                let Js_bytemuck: &[u8] = bytemuck::cast_slice(&Js_f32.as_slice());

                let padded_F_invs: &Vec<Matrix3x3> = &(F_invs
                    .iter()
                    .map(|mat| {
                        let mut cols = [[0.0; 4]; 3];
                        for i in 0..3 {
                            cols[i][0] = mat[(0, i)] as f32;
                            cols[i][1] = mat[(1, i)] as f32;
                            cols[i][2] = mat[(2, i)] as f32;
                        }
                        Matrix3x3 { cols }
                    })
                    .collect());
                let padded_F_invs_bytemuck: &[u8] = bytemuck::cast_slice(&padded_F_invs);

                let A_mul_func = |x: &DVector<Float>| {
                    let x_clone = x.clone();
                    let x32: DVector<f32> = x_clone.map(|a| a as f32);
                    async move {
                        gpu_compute_force_differential(
                            n_vertices,
                            tetrahedra,
                            &-x32,
                            Js_bytemuck,
                            padded_F_invs_bytemuck,
                            wgpu_context,
                        )
                        .await
                        .map(|a| a as Float)
                            + mass_matrix_lumped.component_mul(&x_clone) / dt_squared
                    }
                };
                delta_x = conjugate_gradient(A_mul_func, &b, &dx0, 1e-3).await;
            } else {
                let F_inv_Ts: Vec<Matrix3<Float>> =
                    F_invs.iter().map(|F_inv| F_inv.transpose()).collect();
                let Bs = &self.Bs;
                let Ws = &self.Ws;
                let mu = self.mu;
                let lambda = self.lambda;
                let Js = &Js;
                let F_invs = &F_invs;
                let F_inv_Ts = &F_inv_Ts;

                let A_mul_func = |x: &DVector<Float>| {
                    let x_clone = x.clone();
                    async move {
                        cpu_compute_force_differential(
                            n_vertices, tetrahedra, &-&x_clone, Bs, Ws, mu, lambda, Js, F_invs,
                            F_inv_Ts,
                        )
                        .await
                            + mass_matrix_lumped.component_mul(&x_clone) / dt_squared
                    }
                };
                delta_x = conjugate_gradient(A_mul_func, &b, &dx0, 1e-3).await;
            }

            // TODO: backtracking line-search
            // let mut alpha = 1.0;
            // let E_0 = self.compute_energy(&qdot_new, dt);
            // let mut qdot_update = &qdot_new + alpha * &delta_x / dt;
            // let mut E_update = self.compute_energy(&qdot_update, dt);
            // while E_update > E_0 && alpha > 1e-4 {
            //     console_log("backtrack");
            //     alpha *= 0.5;
            //     qdot_update = &qdot_new + alpha * &delta_x / dt;
            //     E_update = self.compute_energy(&qdot_update, dt);
            // }
            // qdot_new += alpha * &delta_x / dt;
            // q_new += alpha * &delta_x;

            qdot_new += &delta_x / dt;
            q_new += &delta_x;

            i += 1;
        }

        // Assemble contact Jacobian, and formulate contact cone problem
        // Ref: Contact and Friction Simulation for Computer Graphics, 2022,
        //      Section 4.1 Equations of Motion for a Collection of Soft Bodies
        let mut Js: Vec<CscMatrix<Float>> = vec![];
        for i in 0..self.n_vertices {
            let index = i * 3;
            let q = vector![self.q[index], self.q[index + 1], self.q[index + 2]];
            for halfspace in self.halfspaces.iter() {
                if halfspace.has_inside(&q) {
                    // in contact with halfspace
                    let n = halfspace.normal;
                    let t = {
                        let candidate = n.cross(&Vector3::x_axis());
                        if candidate.norm() != 0.0 {
                            UnitVector3::new_normalize(candidate)
                        } else {
                            UnitVector3::new_normalize(n.cross(&Vector3::y_axis()))
                        }
                    };
                    let b = UnitVector3::new_normalize(n.cross(&t));
                    let C = Matrix3::from_rows(&[n.transpose(), t.transpose(), b.transpose()]);
                    let mut coo = CooMatrix::new(3, self.n_vertices * 3);
                    for i in 0..3 {
                        for j in 0..3 {
                            coo.push(i, index + j, C[(i, j)]);
                        }
                    }
                    Js.push(CscMatrix::from(&coo));
                }
            }
        }

        if Js.len() > 0 {
            let mut J: CooMatrix<Float> = CooMatrix::new(3 * Js.len(), self.n_vertices * 3);
            for (index, m) in Js.iter().enumerate() {
                for (i, j, v) in m.triplet_iter() {
                    J.push(index * 3 + i, j, *v);
                }
            }
            let mut J_mul_M_inverse = CscMatrix::from(&J);
            let J = J_mul_M_inverse.clone();
            for (_row, col, val) in J_mul_M_inverse.triplet_iter_mut() {
                *val /= self.mass_matrix_lumped[col];
            }
            let G = &J_mul_M_inverse * &J.transpose();
            let g = &J * &qdot_new;
            let P = ClarabelCscMatrix::new(
                G.nrows(),
                G.ncols(),
                G.col_offsets().to_vec(),
                G.row_indices().to_vec(),
                G.values().to_vec(),
            );
            let lambda = solve_cone_complementarity(&P, &g);
            qdot_new += &J_mul_M_inverse.transpose() * lambda;
        }

        self.qdot = qdot_new;
        self.q += dt * &self.qdot;

        // let internal_force = self.compute_internal_forces(&self.q);
        // // let qddot = self.mass_matrix_cholesky.solve(&(internal_force + tau));
        // let qddot = (internal_force + tau).component_div(&self.mass_matrix_lumped);
        // self.qdot += dt * qddot;
        // self.q += dt * &self.qdot;
    }

    /// Computes the deformation gradient for a single tetrahedron
    /// i.e. returns F = dφ/dq
    fn compute_deformation_gradients(
        q: &DVector<Float>,
        tetrahedron: &Vec<usize>,
        B: &Matrix3<Float>,
    ) -> Matrix3<Float> {
        let i_vi = tetrahedron[0] * 3;
        let i_vj = tetrahedron[1] * 3;
        let i_vk = tetrahedron[2] * 3;
        let i_vl = tetrahedron[3] * 3;

        #[rustfmt::skip]
        let D = Matrix3::<Float>::new(
            q[i_vl] - q[i_vi],          q[i_vl] - q[i_vj],          q[i_vl] - q[i_vk],
            q[i_vl + 1] - q[i_vi + 1],  q[i_vl + 1] - q[i_vj + 1],  q[i_vl + 1] - q[i_vk + 1],
            q[i_vl + 2] - q[i_vi + 2],  q[i_vl + 2] - q[i_vj + 2],  q[i_vl + 2] - q[i_vk + 2],
        );

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

/// CPU version of gpu_compute_force_differential.
/// See gpu_compute_force_differential's doc.
async fn cpu_compute_force_differential(
    n_vertices: usize,
    tetrahedra: &Vec<Vec<usize>>,
    dx: &DVector<Float>,
    Bs: &Vec<Matrix3<Float>>,
    Ws: &Vec<Float>,
    mu: Float,
    lambda: Float,
    Js: &Vec<Float>,
    F_invs: &Vec<Matrix3<Float>>,
    F_inv_Ts: &Vec<Matrix3<Float>>,
) -> DVector<Float> {
    let mut df = DVector::zeros(n_vertices * 3);
    for (tetrahedron, B, volume, J, F_inv, F_inv_T) in izip!(
        tetrahedra.iter(),
        Bs.iter(),
        Ws.iter(),
        Js.iter(),
        F_invs.iter(),
        F_inv_Ts.iter()
    ) {
        let i_vi = tetrahedron[0] * 3;
        let i_vj = tetrahedron[1] * 3;
        let i_vk = tetrahedron[2] * 3;
        let i_vl = tetrahedron[3] * 3;

        #[rustfmt::skip]
        let dD = Matrix3::<Float>::new(
            dx[i_vl] - dx[i_vi],          dx[i_vl] - dx[i_vj],          dx[i_vl] - dx[i_vk],
            dx[i_vl + 1] - dx[i_vi + 1],  dx[i_vl + 1] - dx[i_vj + 1],  dx[i_vl + 1] - dx[i_vk + 1],
            dx[i_vl + 2] - dx[i_vi + 2],  dx[i_vl + 2] - dx[i_vj + 2],  dx[i_vl + 2] - dx[i_vk + 2],
        );

        let dF = dD * B;
        let F_inv_dF = F_inv * dF;
        let dP = mu * dF
            + (mu - lambda * J.ln()) * F_inv_T * F_inv_dF.transpose()
            + lambda * F_inv_dF.trace() * F_inv_T;
        let dH = *volume * dP * B.transpose();

        let dh1 = dH.column(0);
        let dh2 = dH.column(1);
        let dh3 = dH.column(2);

        df.rows_mut(i_vi, 3).add_assign(dh1);
        df.rows_mut(i_vj, 3).add_assign(dh2);
        df.rows_mut(i_vk, 3).add_assign(dh3);
        df.rows_mut(i_vl, 3).add_assign(-dh1 - dh2 - dh3);
    }
    df
}

/// Computes the dF for a given dq at current q.
/// i.e. returns df = -K dq, where K is the stiffness matrix, aka d^2V/dq^2
/// Fs is a vec of deformable gradients, one for each tetrahedron
/// Ref:
///     FEM Simulation of 3D Deformable Solids: A practitioner’s guide to theory, discretization and model reduction.
///     Part One: The classical FEM method and discretization methodology,
///     Eftychios D. Sifakis, 2012, Section 4.3 Force differentials
async fn gpu_compute_force_differential(
    n_vertices: usize,
    tetrahedra: &Vec<Vec<usize>>,
    dx: &DVector<f32>,
    Js_bytemuck: &[u8],
    padded_F_invs_bytemuck: &[u8],
    wgpu_context: &WgpuContext,
) -> DVector<f32> {
    wgpu_context.queue.write_buffer(
        &wgpu_context.input_buffers["dq"],
        0,
        bytemuck::cast_slice(&dx.as_slice()),
    );
    wgpu_context.queue.write_buffer(
        &wgpu_context.input_buffers["F_invs"],
        0,
        padded_F_invs_bytemuck,
    );
    wgpu_context
        .queue
        .write_buffer(&wgpu_context.input_buffers["Js"], 0, Js_bytemuck);

    let dH_vec = compute_dH(&wgpu_context, tetrahedra).await;
    wgpu_context.download_buffers["dH"].unmap();

    let mut df = DVector::zeros(n_vertices * 3);
    for (tetrahedron, dH) in izip!(tetrahedra.iter(), dH_vec.iter()) {
        let i_vi = tetrahedron[0] * 3;
        let i_vj = tetrahedron[1] * 3;
        let i_vk = tetrahedron[2] * 3;
        let i_vl = tetrahedron[3] * 3;

        let dh1 = dH.column(0);
        let dh2 = dH.column(1);
        let dh3 = dH.column(2);

        df.rows_mut(i_vi, 3).add_assign(dh1);
        df.rows_mut(i_vj, 3).add_assign(dh2);
        df.rows_mut(i_vk, 3).add_assign(dh3);
        df.rows_mut(i_vl, 3).add_assign(-dh1 - dh2 - dh3);
    }
    df
}

/// Conjugate Gradient method for solving linear system A x = b, where A is
/// positive-definite.
/// Ref:
///     An Introduction to the Conjugate Gradient Method Without the Agonizing
///     Pain, Jonathan Richard Shewchuk, 1994, Section B2. Conjugate Gradients
async fn conjugate_gradient<F, Fut>(
    A_mul: F,
    b: &DVector<Float>,
    x0: &DVector<Float>,
    abs_tol: Float,
) -> DVector<Float>
where
    F: Fn(&DVector<Float>) -> Fut,
    Fut: Future<Output = DVector<Float>>,
{
    let mut x = x0.clone();

    let mut i = 0;
    let mut r = b - A_mul(x0).await;
    let mut d = r.clone();
    let mut delta_new = r.norm_squared();

    let max_iteration = 50;
    // TODO: guard against or warn on NaN?
    while i < max_iteration && !(delta_new < abs_tol) {
        let q: DVector<Float> = A_mul(&d).await;
        let alpha = delta_new / d.dot(&q);
        x += alpha * &d;
        if (i + 1) % 50 == 0 {
            r = b - A_mul(&x).await;
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

pub async fn read_fem_box() -> FEMDeformable {
    let file_path = "data/box.mesh";
    let buf = read_file(file_path);

    let (vertices, tetrahedra) = read_mesh(&buf);
    FEMDeformable::new(vertices, tetrahedra, 100.0, 6e5, 0.4).await
}

#[cfg(test)]
mod solver_tests {
    use na::{Cholesky, DMatrix, DVector};
    use rand::Rng;

    use crate::{assert_vec_close, types::Float};

    use super::conjugate_gradient;

    #[tokio::test]
    async fn conjugate_gradient_random() {
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
            let A_ref = &A;
            let x_cg = conjugate_gradient(
                |x| {
                    let x_clone = x.clone();
                    async move { A_ref * x_clone }
                },
                &b,
                &x0,
                1e-10,
            )
            .await;

            // Assert
            let x_cholesky = Cholesky::new(A).unwrap().solve(&b);
            assert_vec_close!(x_cg, x_cholesky, 1e-5);
        }
    }
}

#[cfg(test)]
mod fem_deformable_tests {

    use na::dvector;

    use crate::{assert_close, assert_vec_close};

    use super::*;

    /// Stays stationay under no force
    #[tokio::test]
    async fn stationary() {
        // Arrange
        let mut deformable = read_fem_box().await;
        let initial_q = deformable.q.clone();
        let initial_qdot = deformable.qdot.clone();

        // Act
        let final_time = 0.2;
        let dt = 1.0 / 60.0;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            deformable.step(dt, &dvector![]).await;
        }

        // Assert
        assert_vec_close!(initial_q, deformable.q, 1e-2);
        assert_vec_close!(initial_qdot, deformable.qdot, 1e-2);
    }

    /// Constant velocity
    #[tokio::test]
    async fn constant_velocity() {
        // Arrange
        let mut deformable = read_fem_box().await;
        let initial_qdot = DVector::from_element(deformable.n_vertices * 3, 1.0);
        deformable.qdot = initial_qdot.clone();

        // Act
        let final_time = 0.2;
        let dt = 1.0 / 60.0;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            deformable.step(dt, &dvector![]).await;
        }

        // Assert
        assert_vec_close!(initial_qdot, deformable.qdot, 1e-2);
    }

    /// Drag the body by a point, and expect whole body translation
    #[tokio::test]
    async fn drag_at_a_point() {
        // Arrange
        let mut deformable = read_fem_box().await;
        let initial_q = deformable.q.clone();

        // Act
        let direction = vector![1.0, 1.0, 1.0];
        let action_point_index = deformable.extreme_point(&direction);
        let mut tau = DVector::zeros(deformable.n_vertices * 3);
        let force = 1e2;
        tau[action_point_index * 3] = force;
        tau[action_point_index * 3 + 1] = force;
        tau[action_point_index * 3 + 2] = force;

        let final_time = 0.2;
        let dt = 1.0 / 60.0;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            deformable.step(dt, &tau).await;
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

    #[tokio::test]
    async fn drop_on_floor() {
        // Arrange
        let mut deformable = read_fem_box().await;
        let angle = Float::to_radians(0.0);
        let normal = UnitVector3::new_normalize(vector![angle.sin(), 0.0, angle.cos()]);
        let height = -1.05;
        let ground = HalfSpace::new(normal, height);
        deformable.add_halfspace(ground);
        deformable.enable_gravity = true;

        // Act
        let final_time = 0.2;
        let dt = 1.0 / 60.0;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            deformable.step(dt, &dvector![]).await;
        }

        // Assert
        let mut min_z = Float::INFINITY;
        for i in 0..deformable.n_vertices {
            let z = deformable.q[i * 3 + 2];
            min_z = min_z.min(z);
        }
        assert_close!(min_z, height, 1e-2);
    }
}
