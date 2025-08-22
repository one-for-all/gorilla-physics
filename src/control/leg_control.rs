use clarabel::{
    algebra::{CscMatrix, MatrixMathMut},
    solver::{DefaultSettings, DefaultSolver, IPSolver},
};
use na::{vector, DMatrix, Matrix3, Matrix3x6, Vector2, Vector3};

use crate::{
    control::Controller, flog, joint::JointTorque, spatial::spatial_vector::SpatialVector,
    types::Float, util::skew_symmetric, GRAVITY, PI,
};

/// QP based controller with ZMP dynamics
pub struct LegController {}

impl Controller for LegController {
    fn control(
        &mut self,
        state: &mut crate::mechanism::MechanismState,
        input: Option<&super::ControlInput>,
    ) -> Vec<crate::joint::JointTorque> {
        let q1 = state.q[1].float();
        let q2 = state.q[2].float();
        let q = vector![q1, q2];

        let q1_dot = state.v[1].float();
        let q2_dot = state.v[2].float();
        let q_dot = vector![q1_dot, q2_dot];

        let q_des = vector![PI / 4., -PI / 2.];
        let q_ddot_des = (q_des - q) - 0.1 * q_dot;

        let l = 0.2;
        let z_com = 1. / 3.
            * (l / 2. * (PI / 4.).cos() + l * (PI / 4.).cos() + l / 2. * (PI / 4. - PI / 2.).cos());
        flog!("z com: {}", z_com);
        let y_com = 1. / 3. * (-l / 2. * q1.sin() - l * q1.sin() - l / 2. * (q1 + q2).sin());

        // Compute J_com systematically
        let J_spatial_vs = state.spatial_velocity_jacobians();
        let mat_linear_v_com = state.com_linear_velocity_extraction_matrices();

        let T_keep_y = Vector3::y().transpose() * &mat_linear_v_com[2]; // T_v_com;
        let J2_1 = T_keep_y * &J_spatial_vs[1]; // T1_to_world * S1;
        let J2_2 = T_keep_y * &J_spatial_vs[2]; // T2_to_world * S2;
        let J2 = vector![J2_1[(0, 0)], J2_2[(0, 0)]];

        let J1 = vector![
            (Vector3::y().transpose() * &mat_linear_v_com[1] * &J_spatial_vs[1])[(0, 0)],
            0.
        ];

        // flog!("J1: {}", J1);
        // flog!("J2: {}", J2);
        // flog!("J1 diff: {}", J1 - J1_orig);
        // flog!("J2 diff: {}", J2 - J2_orig);
        // flog!("base transform: {}", bodies_to_root[1].iso);
        let J = 1. / 3. * (J1 + J2);

        // Compute J_dot_com systematically
        let J_spatial_v_derivs = state.spatial_velocity_jacobian_derivatives();
        let mat_linear_v_com_derivs = state.com_linear_velocity_extraction_matrix_derivatives();
        let J1_dot = vector![
            (Vector3::<Float>::y().transpose()
                * (&mat_linear_v_com_derivs[1] * &J_spatial_vs[1]
                    + &mat_linear_v_com[1] * &J_spatial_v_derivs[1]))[(0, 0)],
            0.
        ];
        let J2_dot = vector![
            (Vector3::<Float>::y().transpose()
                * (&mat_linear_v_com_derivs[2] * &J_spatial_vs[1]
                    + &mat_linear_v_com[2] * &J_spatial_v_derivs[1]))[(0, 0)],
            (Vector3::<Float>::y().transpose()
                * (&mat_linear_v_com_derivs[2] * &J_spatial_vs[2]
                    + &mat_linear_v_com[2] * &J_spatial_v_derivs[2]))[(0, 0)],
        ];

        // flog!("J1 dot diff: {}", J1_dot - J1_dot_orig);
        // flog!("J2 dot diff: {}", J2_dot - J2_dot_orig);
        let J_dot = 1. / 3. * (J1_dot + J2_dot);

        // Fill in pre-computed Ricatti equation solution S
        let S = DMatrix::from_row_slice(2, 2, &[0.19606829, 0.01922139, 0.01922139, 0.00188435]);
        let B = DMatrix::from_row_slice(2, 1, &[0., 1.]);
        let y_dot_com = J_dot.tr_mul(&q_dot)[(0, 0)];
        let x = vector![y_com, y_dot_com];

        let w_q_ddot = 0.1;
        let P = ((z_com / GRAVITY).powi(2) * J * &J.transpose()
            + DMatrix::identity(2, 2).scale(w_q_ddot))
        .scale(2.);
        let P = CscMatrix::from(P.row_iter());

        let opt_q = ((z_com / GRAVITY).powi(2) * 2. * J_dot.tr_mul(&q_dot)[(0, 0)]
            - 2. * z_com / GRAVITY * y_com
            + 2. * x.tr_mul(&(S * B))[(0, 0)])
            * J
            - 2. * w_q_ddot * q_ddot_des;

        let A = CscMatrix::zeros((0, 2));
        let b = vec![];
        let cones = [];

        let settings = DefaultSettings::default();
        let mut solver = DefaultSolver::new(&P, &opt_q.as_slice(), &A, &b, &cones, settings);

        solver.solve();

        let tau = solver.solution.x;
        flog!(
            "tau diff: {}",
            Vector2::from_column_slice(&tau) - q_ddot_des
        );
        flog!("angle diff: {}", q - q_des);
        // let tau = q_ddot_des.clone();

        vec![
            JointTorque::Spatial(SpatialVector::zero()),
            // JointTorque::None,
            JointTorque::Float(tau[0]),
            JointTorque::Float(tau[1]),
        ]
    }
}
