use itertools::izip;
use na::{
    vector, DMatrix, DVector, Matrix2, Matrix3, Matrix3x2, Matrix3x4, Matrix4x3, RowVector,
    RowVector3, Vector2, Vector3,
};
use nalgebra_sparse::{factorization::CscCholesky, CooMatrix, CscMatrix};

use std::ops::AddAssign;

use crate::{flog, types::Float, util::skew_symmetric, GRAVITY, PI};

/// Compute the area of a triangle, given the lengths of three sides
/// Ref: Heron's formula. https://en.wikipedia.org/wiki/Heron%27s_formula
fn triangle_area(a: Float, b: Float, c: Float) -> Float {
    let s = (a + b + c) / 2.;
    return (s * (s - a) * (s - b) * (s - c)).sqrt();
}

#[cfg(test)]
mod triangle_area_tests {
    use crate::{assert_close, fem::cloth::triangle_area, types::Float};

    #[test]
    fn triangle_area_test1() {
        let a = 1.;
        let b = 1.;
        let c = (2. as Float).sqrt();
        let area = 0.5;

        assert_close!(triangle_area(a, b, c), area, 1e-8);
    }

    #[test]
    fn triangle_area_test2() {
        let a = 1.;
        let b = 1.;
        let c = 1.;
        let area = (3. as Float).sqrt() / 4.;

        assert_close!(triangle_area(a, b, c), area, 1e-8);
    }
}

pub struct Cloth {
    pub vertices: Vec<Vector3<Float>>,
    pub triangles: Vec<[usize; 3]>,

    pub q: DVector<Float>,
    pub qdot: DVector<Float>,

    areas: Vec<Float>,                      // area of each triangle
    M: CscMatrix<Float>,                    // mass matrix
    M_cholesky: CscCholesky<Float>,         // Cholesky factorization of mass matrix
    F_right_factors: Vec<Matrix4x3<Float>>, // the right factor of deformation matrix F
    Bs: Vec<DMatrix<Float>>,
    Ns_stacked: Vec<DMatrix<Float>>,

    P: Option<CscMatrix<Float>>, // Free vertex selection matrix, of size (dof-fixed_dof, dof)
}

impl Cloth {
    pub fn new(vertices: Vec<Vector3<Float>>, triangles: Vec<[usize; 3]>) -> Self {
        let n_vertices = vertices.len();
        let q = DVector::from_iterator(
            n_vertices * 3,
            vertices.iter().flat_map(|v| v.iter().copied()),
        );
        let qdot = DVector::zeros(n_vertices * 3);

        let M_placeholder = CscMatrix::identity(1);
        let M_cholesky = CscCholesky::factor(&M_placeholder).unwrap();
        let mut cloth = Self {
            vertices,
            triangles,
            q,
            qdot,
            areas: vec![],
            M: M_placeholder,
            M_cholesky,
            F_right_factors: vec![],
            Bs: vec![],
            Ns_stacked: vec![],
            P: None,
        };
        cloth.compute_triangle_areas();
        cloth.compute_mass_matrix();
        cloth.compute_deformation_constants();
        cloth
    }

    fn x_at_index(&self, index: usize) -> Vector3<Float> {
        let i = index * 3;
        Vector3::new(self.q[i], self.q[i + 1], self.q[i + 2])
    }

    /// Pre-compute and store the areas of each triangle
    fn compute_triangle_areas(&mut self) {
        assert!(self.areas.is_empty());
        for [v0, v1, v2] in self.triangles.iter() {
            let X0 = self.vertices[*v0];
            let X1 = self.vertices[*v1];
            let X2 = self.vertices[*v2];
            let dX1_X0 = X1 - X0;
            let dX2_X0 = X2 - X0;
            let dX2_X1 = X2 - X1;
            let area = triangle_area(dX1_X0.norm(), dX2_X0.norm(), dX2_X1.norm());
            self.areas.push(area);
        }
    }

    /// Pre-compute and store the mass matrix of the cloth
    fn compute_mass_matrix(&mut self) {
        // Triplets to build up the sparse mass matrix
        let mut triplets = Vec::new();

        for ([v0, v1, v2], area) in izip!(self.triangles.iter(), self.areas.iter()) {
            // Create entries in mass matrix
            // TODO: use realistic density
            let density = 10.0;
            let inertia_off_diag = density * area / 12.0;
            let inertia_diag = density * area / 6.0;
            for v_a in [v0, v1, v2] {
                for v_b in [v0, v1, v2] {
                    let inertia = if v_a == v_b {
                        inertia_diag
                    } else {
                        inertia_off_diag
                    };

                    // insert a Indentity matrix scaled by inertia
                    let irow = 3 * v_a;
                    let icol = 3 * v_b;
                    for i in 0..3 {
                        triplets.push((irow + i, icol + i, inertia));
                    }
                }
            }
        }

        // Build up the mass matrix
        let M = CooMatrix::try_from_triplets_iter(self.q.len(), self.q.len(), triplets).unwrap();
        self.M = CscMatrix::from(&M);
        self.M_cholesky = CscCholesky::factor(&self.M).unwrap();
    }

    /// Pre-compute and store several constant matrices related to deformation.
    /// 1. the right factor of the deformation matrix F
    ///     where F = dx/dX = left * right
    /// 2. B, D matrix rows stacked across
    /// 3. N_stacked, which is normal N stacked diagonally into 3 columns
    fn compute_deformation_constants(&mut self) {
        for [v0, v1, v2] in self.triangles.iter() {
            let X0 = self.vertices[*v0];
            let X1 = self.vertices[*v1];
            let X2 = self.vertices[*v2];
            let dX1_X0 = X1 - X0;
            let dX2_X0 = X2 - X0;
            let N = (dX1_X0).cross(&dX2_X0).normalize();

            let T = Matrix3x2::from_columns(&[dX1_X0, dX2_X0]);
            let T_tr_T = T.tr_mul(&T);
            let tmp = T_tr_T.try_inverse().unwrap() * T.transpose();
            let mut D: Matrix3<Float> = Matrix3::zeros();
            let ones = Vector2::repeat(1.0);
            D.fixed_view_mut::<1, 3>(0, 0)
                .copy_from(&-ones.tr_mul(&tmp));
            D.fixed_view_mut::<2, 3>(1, 0).copy_from(&tmp);

            // compute F right factor
            let mut right = Matrix4x3::zeros();
            right.fixed_view_mut::<3, 3>(0, 0).copy_from(&D);
            right.fixed_view_mut::<1, 3>(3, 0).copy_from(&N.transpose());
            self.F_right_factors.push(right);

            // compute B
            let mut B = DMatrix::zeros(9, 9);
            for i in 0..3 {
                for j in 0..3 {
                    let irow = 3 * i;
                    let icol = i + 3 * j;
                    B.fixed_view_mut::<3, 1>(irow, icol)
                        .copy_from(&D.row(j).transpose());
                }
            }
            self.Bs.push(B);

            // compute N_stacked
            let mut N_stacked = DMatrix::zeros(9, 3);
            for i in 0..3 {
                let irow = i * 3;
                let icol = i;
                N_stacked.fixed_view_mut::<3, 1>(irow, icol).copy_from(&N);
            }
            self.Ns_stacked.push(N_stacked);
        }
    }

    /// Fix the positions of the input vertices
    pub fn fix_vertices(&mut self, fixed_vertices: Vec<usize>) {
        // Create free vertex selection matrix
        let dof = self.q.len() - fixed_vertices.len() * 3;
        let mut P = CooMatrix::zeros(dof, self.q.len());
        let mut irow = 0;
        for v in 0..self.vertices.len() {
            if !fixed_vertices.contains(&v) {
                let icol = 3 * v;
                P.push(irow, icol, 1.);
                P.push(irow + 1, icol + 1, 1.);
                P.push(irow + 2, icol + 2, 1.);
                irow += 3;
            }
        }
        let P = CscMatrix::from(&P);
        self.M = &P * &self.M * P.transpose();
        self.M_cholesky = CscCholesky::factor(&self.M).unwrap();
        self.P = Some(P);
    }

    pub fn step(&mut self, dt: Float) {
        let mut internal_forces: Vec<DVector<Float>> = vec![];
        let mut Hs: Vec<DMatrix<Float>> = vec![]; // Hessians, i.e. d2psi_dq2

        for ([v0, v1, v2], area, F_right_factor, B, N_stacked) in izip!(
            self.triangles.iter(),
            self.areas.iter(),
            self.F_right_factors.iter(),
            self.Bs.iter(),
            self.Ns_stacked.iter(),
        ) {
            let x0 = self.x_at_index(*v0);
            let x1 = self.x_at_index(*v1);
            let x2 = self.x_at_index(*v2);
            let dx1_x0 = x1 - x0;
            let dx2_x0 = x2 - x0;
            let n_tilda = (dx1_x0).cross(&dx2_x0); // un-normalized normal
            let n = n_tilda.normalize();

            let left = Matrix3x4::from_columns(&[x0, x1, x2, n]);

            let F: Matrix3<Float> = left * F_right_factor;
            let svd = F.svd(true, true);
            let U = svd.u.unwrap();
            let S = svd.singular_values;
            let V_T = svd.v_t.unwrap();

            let YM = 1e5; //young's modulus
            let PR = 0.4; //poissons ratio
            let mu = (YM * PR) / ((1.0 + PR) * (1.0 - 2.0 * PR));
            let lambda = YM / (2.0 * (1.0 + PR));

            let tmp = lambda * (S.sum() - 3.);
            let dpsi_ds0 = 2. * mu * (S[0] - 1.) + tmp;
            let dpsi_ds1 = 2. * mu * (S[1] - 1.) + tmp;
            let dpsi_ds2 = 2. * mu * (S[2] - 1.) + tmp;
            let dpsi_dS = Matrix3::from_diagonal(&vector![dpsi_ds0, dpsi_ds1, dpsi_ds2]);

            let dpsi_dF = U * dpsi_dS * V_T;

            // Next, compute dF_dq
            let dn_dn_tilda: Matrix3<Float> =
                1. / n_tilda.norm() * (Matrix3::identity() - n * n.transpose());
            let dn_tilda_ddx2 = skew_symmetric(&dx1_x0);
            let mut ddx2_dq = DMatrix::<Float>::zeros(3, 9);
            ddx2_dq
                .fixed_view_mut::<3, 3>(0, 0)
                .copy_from(&-Matrix3::identity());
            ddx2_dq
                .fixed_view_mut::<3, 3>(0, 6)
                .copy_from(&Matrix3::identity());
            let dn_tilda_ddx1 = -skew_symmetric(&dx2_x0);
            let mut ddx1_dq = DMatrix::<Float>::zeros(3, 9);
            ddx1_dq
                .fixed_view_mut::<3, 3>(0, 0)
                .copy_from(&-Matrix3::identity());
            ddx1_dq
                .fixed_view_mut::<3, 3>(0, 3)
                .copy_from(&Matrix3::identity());
            let dn_tilda_dq = dn_tilda_ddx2 * ddx2_dq + dn_tilda_ddx1 * ddx1_dq;
            let dn_dq = dn_dn_tilda * dn_tilda_dq;
            let dF_dq: DMatrix<Float> = B + N_stacked * dn_dq;

            let mut dpsi_dF_flatten = DVector::<Float>::zeros(9);
            for i in 0..3 {
                for j in 0..3 {
                    dpsi_dF_flatten[i * 3 + j] = dpsi_dF[(i, j)];
                }
            }

            let dpsi_dq: DVector<Float> = *area * dF_dq.tr_mul(&dpsi_dF_flatten);
            internal_forces.push(-dpsi_dq);

            // Compute second derivative of energy w/ respect to q
            // from: dpsi/dq = dF/dq.T * dpsi/dF
            // => d2psi/dq2 = d(dpsi/dq)/dq = dF/dq.T * d2psi/dF2 * dF/dq + d2F/dq2.T * dpsi/dF
            // Note, we ignore the d2F/dq2 part, which is from F's dependence on normal n, and assumed to be negligible.
            let mut d2psi_dF2: DMatrix<Float> = DMatrix::zeros(9, 9);

            let d2psi_ds02 = 2. * mu + lambda;
            let d2psi_ds0ds1 = lambda;
            #[rustfmt::skip]
            let d2psi_dS2 = Matrix3::new(
                d2psi_ds02, d2psi_ds0ds1, d2psi_ds0ds1,
                d2psi_ds0ds1, d2psi_ds02, d2psi_ds0ds1,
                d2psi_ds0ds1, d2psi_ds0ds1, d2psi_ds02
            );

            // Compute jacobian of SVD w/ respect to F
            // [i][j] entries are the jacobians w/ respect to F[i][j]
            let mut dS: Vec<Vec<Vector3<Float>>> = vec![vec![Vector3::zeros(); 3]; 3];
            let mut dV_T: Vec<Vec<Matrix3<Float>>> = vec![vec![Matrix3::zeros(); 3]; 3];
            let mut dU: Vec<Vec<Matrix3<Float>>> = vec![vec![Matrix3::zeros(); 3]; 3];
            let mut d01 = 1. / (S[1] * S[1] - S[0] * S[0]);
            let mut d02 = 1. / (S[2] * S[2] - S[0] * S[0]);
            let mut d12 = 1. / (S[2] * S[2] - S[1] * S[1]);
            // corresponding to conservative solution --- if singularity is detected no angular velocity
            if d01.is_infinite() {
                d01 = 0.;
            }
            if d02.is_infinite() {
                d02 = 0.;
            }
            if d12.is_infinite() {
                d12 = 0.;
            }
            for irow in 0..3 {
                for icol in 0..3 {
                    // Compute dS/dF
                    let U_irow = U.row(irow);
                    let V_T_icol = V_T.column(icol);
                    let mut UVT = U_irow.transpose() * V_T_icol.transpose();
                    dS[irow][icol] = UVT.diagonal();

                    // Compute dV/dF
                    let dS_mat = Matrix3::from_diagonal(&UVT.diagonal());
                    UVT -= dS_mat;
                    let S_mat = Matrix3::from_diagonal(&S);
                    let tmp = S_mat * UVT + UVT.transpose() * S_mat;
                    let w01 = tmp[(0, 1)] * d01;
                    let w02 = tmp[(0, 2)] * d02;
                    let w12 = tmp[(1, 2)] * d12;
                    #[rustfmt::skip]
                    let tmp = Matrix3::new(
                        0.0,    w01,   w02,
                        -w01,    0.0,    w12,
                        -w02,   -w12,    0.0
                    );
                    dV_T[irow][icol] = tmp.tr_mul(&V_T);

                    // Compute dU/dF
                    let tmp = UVT * S_mat + S_mat * UVT.transpose();
                    let w01 = tmp[(0, 1)] * d01;
                    let w02 = tmp[(0, 2)] * d02;
                    let w12 = tmp[(1, 2)] * d12;
                    #[rustfmt::skip]
                    let tmp = Matrix3::new(
                        0.0,    w01,   w02,
                        -w01,    0.0,    w12,
                        -w02,   -w12,    0.0
                    );
                    dU[irow][icol] = U * tmp;

                    // Compute d2psi_dF2
                    let d2psi_dSdF = d2psi_dS2 * dS[irow][icol];
                    let d2psi_dFij = dU[irow][icol] * dpsi_dS * V_T
                        + U * Matrix3::from_diagonal(&d2psi_dSdF) * V_T
                        + U * dpsi_dS * dV_T[irow][icol];
                    let mut d2psi_dFij_flatten = DVector::<Float>::zeros(9);
                    for i in 0..3 {
                        for j in 0..3 {
                            d2psi_dFij_flatten[i * 3 + j] = d2psi_dFij[(i, j)];
                        }
                    }
                    d2psi_dF2
                        .row_mut(irow * 3 + icol)
                        .copy_from(&d2psi_dFij_flatten.transpose());
                }
            }

            Hs.push(dF_dq.transpose() * d2psi_dF2 * dF_dq);
        }

        // let mut dense = DMatrix::<Float>::zeros(M.nrows(), M.ncols());
        // for (row, col, val) in M.triplet_iter() {
        //     dense[(row, col)] = *val;
        // }

        // flog!("internal f:");
        // for f in internal_forces.iter() {
        //     flog!("{}", f);
        // }

        // Assemble stiffness matrix, K = -Hessian
        let mut K = CooMatrix::new(self.q.len(), self.q.len());
        for ([v0, v1, v2], H) in izip!(self.triangles.iter(), Hs.iter()) {
            for (i, v_a) in [v0, v1, v2].iter().enumerate() {
                for (j, v_b) in [v0, v1, v2].iter().enumerate() {
                    let irow = 3 * *v_a;
                    let icol = 3 * *v_b;
                    // copy the sub 3x3 matrix in Hs
                    for row in 0..3 {
                        for col in 0..3 {
                            K.push(irow + row, icol + col, -H[(3 * i + row, 3 * j + col)]);
                        }
                    }
                }
            }
        }
        let mut K = CscMatrix::from(&K);
        if let Some(P) = &self.P {
            K = P * K * P.transpose();
        }

        let mut total_force = DVector::zeros(self.q.len());
        for ([v0, v1, v2], f) in izip!(self.triangles.iter(), internal_forces.iter()) {
            total_force
                .fixed_rows_mut::<3>(3 * v0)
                .add_assign(&f.fixed_rows::<3>(0));
            total_force
                .fixed_rows_mut::<3>(3 * v1)
                .add_assign(&f.fixed_rows::<3>(3));
            total_force
                .fixed_rows_mut::<3>(3 * v2)
                .add_assign(&f.fixed_rows::<3>(6));
        }

        // Modified M and force to take into account of fixed nodes
        if let Some(P) = &self.P {
            total_force = P * total_force;
        }

        let mut gravity_acc = DVector::zeros(self.q.len());
        for i in 0..self.vertices.len() {
            let index = 3 * i + 2;
            gravity_acc[index] = -GRAVITY;
        }
        if let Some(P) = &self.P {
            gravity_acc = P * gravity_acc;
        }
        let gravity_force = &self.M * gravity_acc;
        total_force += &gravity_force;

        // let mut qddot: DMatrix<Float> = self.M_cholesky.solve(&total_force);
        // // Transform qddot back to origial generalized coordinates space
        // if let Some(P) = &self.P {
        //     qddot = P.transpose() * &qddot;
        // }
        // self.qdot += &qddot * dt;

        let qdot = if let Some(P) = &self.P {
            P * &self.qdot
        } else {
            self.qdot.clone()
        };

        let A = &self.M - dt * dt * &K;
        let A_cholesky = CscCholesky::factor(&A).unwrap();
        let b = &self.M * qdot + total_force * dt;
        let qdot_new = A_cholesky.solve(&b);
        let mut qdot_new: DVector<Float> = DVector::from_column_slice(qdot_new.data.as_slice());
        if let Some(P) = &self.P {
            qdot_new = P.transpose() * qdot_new;
        }

        self.qdot = qdot_new;
        self.q += &self.qdot * dt;
    }
}

#[cfg(test)]
mod cloth_tests {
    use na::{vector, DVector, UnitQuaternion, Vector3};

    use crate::{
        assert_vec_close, builders::cloth_builder::build_cloth, fem::cloth::Cloth, types::Float, PI,
    };

    #[test]
    fn one_triangle_cloth_test() {
        // Arrange
        let vertices = vec![
            vector![0., 0., 0.],
            vector![1., 0., 0.],
            vector![0., 0., -1.],
        ];
        let triangles = vec![[0, 1, 2]];
        let mut cloth = Cloth::new(vertices.clone(), triangles);
        cloth.fix_vertices(vec![0, 1]);

        // Act
        let final_time = 1.0;
        let dt = 1e-2;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            cloth.step(dt);
        }

        // Assert
        let q = &cloth.q;
        assert_eq!(q.fixed_rows::<3>(0), vertices[0]);
        assert_eq!(q.fixed_rows::<3>(3), vertices[1]);
        assert_ne!(q.fixed_rows::<3>(6), vertices[2]);
        assert_vec_close!(q.fixed_rows::<3>(6), vertices[2], 1e-3);

        let qdot = &cloth.qdot;
        assert_eq!(qdot.fixed_rows::<3>(0), Vector3::zeros());
        assert_eq!(qdot.fixed_rows::<3>(3), Vector3::zeros());
        assert_ne!(qdot.fixed_rows::<3>(6), Vector3::zeros());
        assert_vec_close!(qdot.fixed_rows::<3>(6), Vector3::<Float>::zeros(), 1e-2);
    }

    #[test]
    fn two_triangle_cloth_test() {
        // Arrange
        let vertices = vec![
            vector![0., 0., 0.],
            vector![1., 0., 0.],
            vector![0., 0., -1.],
            vector![1., 0., -1.],
        ];
        let triangles = vec![[0, 1, 2], [1, 3, 2]];
        let mut cloth = Cloth::new(vertices.clone(), triangles);
        cloth.fix_vertices(vec![0, 1]);

        // Act
        let final_time = 1.0;
        let dt = 1e-2;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            cloth.step(dt);
        }

        // Assert
        let q = &cloth.q;
        assert_eq!(q.fixed_rows::<3>(0), vertices[0]);
        assert_eq!(q.fixed_rows::<3>(3), vertices[1]);

        assert_ne!(q.fixed_rows::<3>(6), vertices[2]);
        assert_vec_close!(q.fixed_rows::<3>(6), vertices[2], 1e-3);
        assert_ne!(q.fixed_rows::<3>(9), vertices[3]);
        assert_vec_close!(q.fixed_rows::<3>(9), vertices[3], 1e-3);

        let qdot = &cloth.qdot;
        assert_eq!(qdot.fixed_rows::<3>(0), Vector3::zeros());
        assert_eq!(qdot.fixed_rows::<3>(3), Vector3::zeros());

        assert_ne!(qdot.fixed_rows::<3>(6), Vector3::zeros());
        assert_vec_close!(qdot.fixed_rows::<3>(6), Vector3::<Float>::zeros(), 6e-2);
        assert_ne!(qdot.fixed_rows::<3>(9), Vector3::zeros());
        assert_vec_close!(qdot.fixed_rows::<3>(9), Vector3::<Float>::zeros(), 6e-2);
    }

    #[test]
    fn swing_cloth_test() {
        // Arrange
        let m = 6;
        let n = 6;
        let mut cloth = build_cloth(
            m,
            n,
            0.5,
            UnitQuaternion::from_axis_angle(&Vector3::x_axis(), -PI / 4.),
        );
        cloth.fix_vertices(Vec::from_iter(0..m));

        // Act
        let final_time = 1.;
        let dt = 1e-2;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            cloth.step(dt);
        }

        // Assert
        for i in 0..m {
            let z = cloth.q[3 * i + 2];
            assert!(z == 0.);
        }
        for i in m..cloth.vertices.len() {
            let z = cloth.q[3 * i + 2];
            assert!(z < 0.);
        }
    }
}
