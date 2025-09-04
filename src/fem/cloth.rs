use na::{
    vector, DMatrix, DVector, Matrix2, Matrix3, Matrix3x2, Matrix3x4, Matrix4x3, Vector2, Vector3,
};

use crate::{flog, types::Float, util::skew_symmetric};

pub struct Cloth {
    pub vertices: Vec<Vector3<Float>>,
    pub triangles: Vec<Vec<usize>>,

    pub q: DVector<Float>,
    pub qdot: DVector<Float>,
}

impl Cloth {
    pub fn new(vertices: Vec<Vector3<Float>>, triangles: Vec<Vec<usize>>) -> Self {
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
        let x0 = self.x_at_index(0);
        let x1 = self.x_at_index(1);
        let x2 = self.x_at_index(2);
        let dx1_x0 = x1 - x0;
        let dx2_x0 = x2 - x0;
        let n_tilda = (dx1_x0).cross(&dx2_x0); // un-normalized normal
        let n = n_tilda.normalize();

        let left = Matrix3x4::from_columns(&[x0, x1, x2, n]);

        let X0 = self.vertices[0];
        let X1 = self.vertices[1];
        let X2 = self.vertices[2];
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

        // flog!("left: {}", left);
        // flog!("right: {}", right);

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
                let icol = 3 * j;
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
            1. / (n_tilda.norm()) * (Matrix3::identity() - n * n.transpose());
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

        // TODO: use area
        let dV_dq: DVector<Float> = dpsi_dF_flatten.tr_mul(&dF_dq).transpose();

        // TODO: use mass
        self.qdot += -dV_dq * dt;
        self.q += &self.qdot * dt;
    }
}

#[cfg(test)]
mod cloth_tests {
    use na::{vector, Vector3};

    use crate::fem::cloth::Cloth;

    #[test]
    fn minimal_cloth_test() {
        // Arrange
        let vertices = vec![
            vector![0., 0., 0.],
            vector![1., 0., 0.],
            vector![0., 1., 0.],
            // vector![1., 1., 0.],
        ];
        let triangles = vec![
            vec![0, 1, 2],
            // vec![1, 3, 2]
        ];
        let mut cloth = Cloth::new(vertices, triangles);
        cloth.q[0] = 1e-4;

        // Act
        let final_time = 10.0;
        let dt = 1e-3;
        let num_steps = (final_time / dt) as usize;
        for _s in 0..num_steps {
            cloth.step(dt);
        }

        // Assert
        println!("q: {}", cloth.q);
        println!("qdot: {}", cloth.qdot);
    }
}
