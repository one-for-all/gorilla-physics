use itertools::izip;
use na::{
    vector, DMatrix, DVector, Matrix2, Matrix3, Matrix3x2, Matrix3x4, Matrix4x3, Vector2, Vector3,
};
use nalgebra_sparse::{factorization::CscCholesky, CooMatrix, CscMatrix};

use std::ops::AddAssign;

use crate::{flog, types::Float, util::skew_symmetric, GRAVITY};

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
}

impl Cloth {
    pub fn new(vertices: Vec<Vector3<Float>>, triangles: Vec<[usize; 3]>) -> Self {
        let n_vertices = vertices.len();
        let q = DVector::from_iterator(
            n_vertices * 3,
            vertices.iter().flat_map(|v| v.iter().copied()),
        );
        let qdot = DVector::zeros(n_vertices * 3);

        Self {
            vertices,
            triangles,
            q,
            qdot,
        }
    }

    fn x_at_index(&self, index: usize) -> Vector3<Float> {
        let i = index * 3;
        Vector3::new(self.q[i], self.q[i + 1], self.q[i + 2])
    }

    pub fn step(&mut self, dt: Float) {
        let mut internal_forces: Vec<DVector<Float>> = vec![];

        // Triplets to build up the sparse mass matrix
        let mut triplets = Vec::new();

        for [v0, v1, v2] in self.triangles.iter() {
            let x0 = self.x_at_index(*v0);
            let x1 = self.x_at_index(*v1);
            let x2 = self.x_at_index(*v2);
            let dx1_x0 = x1 - x0;
            let dx2_x0 = x2 - x0;
            let n_tilda = (dx1_x0).cross(&dx2_x0); // un-normalized normal
            let n = n_tilda.normalize();

            let left = Matrix3x4::from_columns(&[x0, x1, x2, n]);

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

            let mut right = Matrix4x3::zeros();
            right.fixed_view_mut::<3, 3>(0, 0).copy_from(&D);
            right.fixed_view_mut::<1, 3>(3, 0).copy_from(&N.transpose());

            let F: Matrix3<Float> = left * right;
            let svd = F.svd(true, true);
            let U = svd.u.unwrap();
            let V_T = svd.v_t.unwrap();
            let S = svd.singular_values;

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
            let mut B = DMatrix::zeros(9, 9);
            for i in 0..3 {
                for j in 0..3 {
                    let irow = 3 * i;
                    let icol = i + 3 * j;
                    B.fixed_view_mut::<3, 1>(irow, icol)
                        .copy_from(&D.row(j).transpose());
                }
            }

            let mut N_ = DMatrix::zeros(9, 3);
            for i in 0..3 {
                let irow = i * 3;
                let icol = i;
                N_.fixed_view_mut::<3, 1>(irow, icol).copy_from(&N);
            }

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
            let dF_dq = B + N_ * dn_dq;

            let mut dpsi_dF_flatten = DVector::<Float>::zeros(9);
            for i in 0..3 {
                for j in 0..3 {
                    dpsi_dF_flatten[i * 3 + j] = dpsi_dF[(i, j)];
                }
            }

            let area = triangle_area(dX1_X0.norm(), dX2_X0.norm(), (X2 - X1).norm());
            let dV_dq: DVector<Float> = area * dpsi_dF_flatten.tr_mul(&dF_dq).transpose();

            internal_forces.push(-dV_dq);

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
        let M = CscMatrix::from(&M);

        // let mut dense = DMatrix::<Float>::zeros(M.nrows(), M.ncols());
        // for (row, col, val) in M.triplet_iter() {
        //     dense[(row, col)] = *val;
        // }

        // flog!("internal f:");
        // for f in internal_forces.iter() {
        //     flog!("{}", f);
        // }

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

        let mut gravity_acc = DVector::zeros(self.q.len());
        for i in 0..self.vertices.len() {
            let index = 3 * i + 2;
            gravity_acc[index] = -GRAVITY;
        }
        let gravity_force = &M * gravity_acc;

        total_force += &gravity_force;

        // Create free node selection matrix
        let m = 6;
        let dof = self.q.len() - m * 3;
        let mut P = CooMatrix::zeros(dof, self.q.len());
        for (i, j) in izip!(0..dof, m * 3..self.q.len()) {
            P.push(i, j, 1.);
        }
        let P = CscMatrix::from(&P);

        // Modified M and force to take into account of fixed nodes
        let M = &P * &M * &P.transpose();
        let total_force = &P * total_force;

        let M_cholesky = CscCholesky::factor(&M).unwrap();
        let qddot: DMatrix<Float> = M_cholesky.solve(&total_force);

        // Transform qddot back to origial generalized coordinates space
        let qddot = &P.transpose() * &qddot;

        self.qdot += &qddot * dt;
        self.q += &self.qdot * dt;
    }
}

#[cfg(test)]
mod cloth_tests {
    use na::{vector, DVector, Vector3};

    use crate::{assert_vec_close, fem::cloth::Cloth, types::Float};

    // TODO: Account for fixed point
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

        // Act
        let final_time = 1.0;
        let dt = 1e-3;
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

        // Act
        let final_time = 1.0;
        let dt = 1e-3;
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
        assert_vec_close!(qdot.fixed_rows::<3>(6), Vector3::<Float>::zeros(), 2e-2);
        assert_ne!(qdot.fixed_rows::<3>(9), Vector3::zeros());
        assert_vec_close!(qdot.fixed_rows::<3>(9), Vector3::<Float>::zeros(), 2e-2);
    }
}
