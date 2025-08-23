use clarabel::{
    algebra::CscMatrix,
    solver::{DefaultSettings, DefaultSolver, IPSolver, SupportedConeT::ZeroConeT},
};
use na::Vector3;
use na::{vector, DMatrix, DVector, Matrix1xX, Matrix6xX, Vector6};

use crate::{
    control::Controller,
    flog,
    joint::{JointTorque, ToFloatDVec},
    spatial::spatial_vector::SpatialVector,
    types::Float,
    GRAVITY, PI,
};

/// QP based controller with ZMP dynamics
pub struct LegController {}

impl Controller for LegController {
    fn control(
        &mut self,
        state: &mut crate::mechanism::MechanismState,
        _input: Option<&super::ControlInput>,
    ) -> Vec<crate::joint::JointTorque> {
        let q1 = state.q[1].float();
        let q2 = state.q[2].float();
        let q = vector![q1, q2];

        let v = state.v.to_float_dvec();
        let dof_robot = v.len();

        let q_des = vector![PI / 4., -PI / 2.];
        let v_dot_des = (q_des - q) - 0.1 * vector![v[6], v[7]];
        let v_dot_des = vector![0., 0., 0., 0., 0., 0., v_dot_des[0], v_dot_des[1]];

        let l = 0.2;
        let z_com = 1. / 3.
            * (l / 2. * (PI / 4.).cos() + l * (PI / 4.).cos() + l / 2. * (PI / 4. - PI / 2.).cos());
        // flog!("z com: {}", z_com);
        let y_com = 1. / 3. * (-l / 2. * q1.sin() - l * q1.sin() - l / 2. * (q1 + q2).sin());

        let n_contacts = 4;
        let dof_contact = n_contacts * 3;
        let dof = dof_robot + dof_contact;

        // Compute J_com systematically
        let J_spatial_vs = state.spatial_velocity_jacobians();
        let mat_linear_v_com = state.com_linear_velocity_extraction_matrices();

        let mut J0: Matrix1xX<Float> = Matrix1xX::zeros(dof_robot);
        J0.fixed_view_mut::<1, 6>(0, 0)
            .copy_from(&(Vector3::y().transpose() * &mat_linear_v_com[0] * &J_spatial_vs[0]));

        let mut J1: Matrix1xX<Float> = Matrix1xX::zeros(dof_robot);
        J1.fixed_view_mut::<1, 6>(0, 0)
            .copy_from(&(Vector3::y().transpose() * &mat_linear_v_com[1] * &J_spatial_vs[0]));
        J1.fixed_view_mut::<1, 1>(0, 6)
            .copy_from(&(Vector3::y().transpose() * &mat_linear_v_com[1] * &J_spatial_vs[1]));

        let mut J2 = Matrix1xX::zeros(dof_robot);
        J2.fixed_view_mut::<1, 6>(0, 0)
            .copy_from(&(Vector3::y().transpose() * &mat_linear_v_com[2] * &J_spatial_vs[0]));
        J2.fixed_view_mut::<1, 1>(0, 6)
            .copy_from(&(Vector3::y().transpose() * &mat_linear_v_com[2] * &J_spatial_vs[1]));
        J2.fixed_view_mut::<1, 1>(0, 7)
            .copy_from(&(Vector3::y().transpose() * &mat_linear_v_com[2] * &J_spatial_vs[2]));

        // flog!("J1: {}", J1);
        // flog!("J2: {}", J2);
        // flog!("J1 diff: {}", J1 - J1_orig);
        // flog!("J2 diff: {}", J2 - J2_orig);
        // flog!("base transform: {}", bodies_to_root[1].iso);
        let J = 1. / 3. * (J0 + J1 + J2);

        // Compute J_dot_com systematically
        let J_spatial_v_derivs = state.spatial_velocity_jacobian_derivatives();
        let mat_linear_v_com_derivs = state.com_linear_velocity_extraction_matrix_derivatives();

        let mut J0_dot: Matrix1xX<Float> = Matrix1xX::zeros(dof_robot);
        J0_dot.fixed_view_mut::<1, 6>(0, 0).copy_from(
            &(Vector3::<Float>::y().transpose()
                * (&mat_linear_v_com_derivs[0] * &J_spatial_vs[0]
                    + &mat_linear_v_com[0] * &J_spatial_v_derivs[0])),
        );

        let mut J1_dot = Matrix1xX::zeros(dof_robot);
        J1_dot.fixed_view_mut::<1, 6>(0, 0).copy_from(
            &(Vector3::<Float>::y().transpose()
                * (&mat_linear_v_com_derivs[1] * &J_spatial_vs[0]
                    + &mat_linear_v_com[1] * &J_spatial_v_derivs[0])),
        );
        J1_dot.fixed_view_mut::<1, 1>(0, 6).copy_from(
            &(Vector3::<Float>::y().transpose()
                * (&mat_linear_v_com_derivs[1] * &J_spatial_vs[1]
                    + &mat_linear_v_com[1] * &J_spatial_v_derivs[1])),
        );

        let mut J2_dot = Matrix1xX::zeros(dof_robot);
        J2_dot.fixed_view_mut::<1, 6>(0, 0).copy_from(
            &(Vector3::<Float>::y().transpose()
                * (&mat_linear_v_com_derivs[2] * &J_spatial_vs[0]
                    + &mat_linear_v_com[2] * &J_spatial_v_derivs[0])),
        );
        J2_dot.fixed_view_mut::<1, 1>(0, 6).copy_from(
            &(Vector3::<Float>::y().transpose()
                * (&mat_linear_v_com_derivs[2] * &J_spatial_vs[1]
                    + &mat_linear_v_com[2] * &J_spatial_v_derivs[1])),
        );
        J2_dot.fixed_view_mut::<1, 1>(0, 7).copy_from(
            &(Vector3::<Float>::y().transpose()
                * (&mat_linear_v_com_derivs[2] * &J_spatial_vs[2]
                    + &mat_linear_v_com[2] * &J_spatial_v_derivs[2])),
        );

        let J_dot = 1. / 3. * (J0_dot + J1_dot + J2_dot);

        // Fill in pre-computed Ricatti equation solution S
        let S = DMatrix::from_row_slice(2, 2, &[0.19606829, 0.01922139, 0.01922139, 0.00188435]);
        let B = DMatrix::from_row_slice(2, 1, &[0., 1.]);
        let y_dot_com = (&J_dot * &v)[(0, 0)];
        let x = vector![y_com, y_dot_com];

        let w_v_dot = 0.1;
        let P = ((z_com / GRAVITY).powi(2) * J.transpose() * &J
            + DMatrix::identity(dof_robot, dof_robot).scale(w_v_dot))
        .scale(2.);
        let mut P_padded = DMatrix::zeros(dof, dof);
        P_padded
            .view_mut((0, 0), (dof_robot, dof_robot))
            .copy_from(&P);
        let P_padded = CscMatrix::from(P_padded.row_iter());

        let opt_q = ((z_com / GRAVITY).powi(2) * 2. * (&J_dot * &v)[(0, 0)]
            - 2. * z_com / GRAVITY * y_com
            + 2. * x.tr_mul(&(S * B))[(0, 0)])
            * J.transpose()
            - 2. * w_v_dot * v_dot_des;
        let mut opt_q_padded = DVector::zeros(dof);
        opt_q_padded
            .view_mut((0, 0), (dof_robot, 1))
            .copy_from(&opt_q);

        // No-slip constraint on base(foot) body.
        // TODO: constrain only planar movement
        let A = Matrix6xX::<Float>::identity(dof);
        let b = Vector6::zeros();
        let cones = [ZeroConeT(6)];

        let A = CscMatrix::from(A.row_iter());

        let settings = DefaultSettings::default();
        let mut solver = DefaultSolver::new(
            &P_padded,
            &opt_q_padded.as_slice(),
            &A,
            &b.as_slice(),
            &cones,
            settings,
        );

        solver.solve();

        let tau = solver.solution.x;
        flog!("tau: {:?}", tau);
        // flog!("tau diff: {}", DVector::from_column_slice(&tau) - v_dot_des);
        // flog!("angle diff: {}", q - q_des);
        // let tau = q_ddot_des.clone();

        vec![
            JointTorque::Spatial(SpatialVector::zero()),
            // JointTorque::None,
            JointTorque::Float(tau[6]),
            JointTorque::Float(tau[7]),
        ]
    }
}
