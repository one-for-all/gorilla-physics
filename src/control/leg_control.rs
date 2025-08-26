use std::collections::HashMap;

use clarabel::{
    algebra::CscMatrix,
    solver::{
        DefaultSettings, DefaultSolver, IPSolver,
        SupportedConeT::{SecondOrderConeT, ZeroConeT},
    },
};
use itertools::izip;
use na::{vector, zero, DMatrix, DVector, Matrix, Matrix1xX, Matrix6xX, Vector6};
use na::{Matrix4x3, Vector3};

use crate::{
    control::Controller,
    dynamics::dynamics_bias,
    flog,
    joint::{JointTorque, ToFloatDVec},
    mechanism::mass_matrix,
    spatial::{spatial_vector::SpatialVector, twist::compute_joint_twists, wrench},
    types::Float,
    util::skew_symmetric,
    GRAVITY, PI,
};

/// QP based controller with ZMP dynamics
/// Ref: An Efficiently Solvable Quadratic Program for Stabilizing Dynamic Locomotion, Scott Kuindersma and etc., 2014
pub struct LegFromFootController {}

impl Controller for LegFromFootController {
    fn control(
        &mut self,
        state: &mut crate::mechanism::MechanismState,
        _input: Option<&super::ControlInput>,
    ) -> Vec<crate::joint::JointTorque> {
        let q_full = state.q.to_float_dvec();
        let q_actuated: DVector<Float> =
            DVector::from_iterator(q_full.len() - 7, q_full.iter().skip(7).cloned());

        let v_full = state.v.to_float_dvec();
        let dof_robot = v_full.len();
        let dof_unactuated = 6;
        let dof_actuated = dof_robot - dof_unactuated;
        let v_actuated =
            DVector::from_iterator(dof_actuated, v_full.iter().skip(dof_unactuated).cloned());

        let q_actuated_des = vector![PI / 4., -PI / 2., PI / 4., 0., 0.];
        let v_dot_des_actuated = (q_actuated_des - &q_actuated) - 1. * v_actuated;
        let mut v_dot_des = DVector::zeros(dof_robot);
        v_dot_des
            .view_mut((dof_unactuated, 0), (dof_actuated, 1))
            .copy_from(&v_dot_des_actuated);

        let l = 0.2;
        // let z_com = 1. / 3.
        //     * (l / 2. * (PI / 4.).cos() + l * (PI / 4.).cos() + l / 2. * (PI / 4. - PI / 2.).cos());
        // let z_com = 0.09428090415820635; // up to thigh
        // let z_com = 0.14402571247741597; // up to hip
        // let z_com = 0.18970562748477146; // up to pelvis
        let z_com = 0.23856180831641274; // pre-computed z_com at nominal q
                                         // flog!("z com: {}", z_com);

        // let y_com =
        //     1. / 3. * (-l / 2. * q1.sin() - l * q1.sin() - l / 2. * (q1 + q2).sin()) + q_full[5];
        let com = state.center_of_mass();
        let y_com = com.y;

        let n_contacts = 4;
        let dof_contact = n_contacts * 3;
        let dof = dof_robot + dof_contact;

        // Compute J_com systematically
        let J_spatial_vs = state.spatial_velocity_jacobians();
        let mat_linear_v_com = state.com_linear_velocity_extraction_matrices();

        let n_bodies = state.bodies.len();
        let joint_dofs = state.joint_dofs();
        // Jacobian of center-of-mass of each body
        let J_coms: Vec<Matrix1xX<Float>> = (0..n_bodies)
            .map(|i| {
                let mut J_com: Matrix1xX<Float> = Matrix1xX::zeros(dof_robot);
                let mut col_offset = 0;
                for j in 0..=i {
                    let joint_dof = joint_dofs[j];
                    J_com.view_mut((0, col_offset), (1, joint_dof)).copy_from(
                        &(Vector3::y().transpose() * &mat_linear_v_com[i] * &J_spatial_vs[j]),
                    );
                    col_offset += joint_dof;
                }
                J_com
            })
            .collect();
        // Jacobian of center-of-mass of the whole system
        let J_com = izip!(state.bodies.iter(), J_coms.iter())
            .map(|(b, J)| b.inertia.mass * J)
            .fold(Matrix1xX::<Float>::zeros(dof_robot), |acc, J| acc + J)
            / state.total_mass();

        // Compute J_dot_com systematically
        let J_spatial_v_derivs = state.spatial_velocity_jacobian_derivatives();
        let mat_linear_v_com_derivs = state.com_linear_velocity_extraction_matrix_derivatives();

        // time derivative of Jacobian of center-of-mass of each body
        let J_dot_coms: Vec<Matrix1xX<Float>> = (0..n_bodies)
            .map(|i| {
                let mut J_dot_com: Matrix1xX<Float> = Matrix1xX::zeros(dof_robot);
                let mut col_offset = 0;
                for j in 0..=i {
                    let joint_dof = joint_dofs[j];
                    J_dot_com
                        .view_mut((0, col_offset), (1, joint_dof))
                        .copy_from(
                            &(Vector3::<Float>::y().transpose()
                                * (&mat_linear_v_com_derivs[i] * &J_spatial_vs[j]
                                    + &mat_linear_v_com[i] * &J_spatial_v_derivs[j])),
                        );
                    col_offset += joint_dof;
                }
                J_dot_com
            })
            .collect();
        // time derivative of Jacobian of center-of-mass of the whole system
        let J_dot_com = izip!(state.bodies.iter(), J_dot_coms.iter())
            .map(|(b, J)| b.inertia.mass * J)
            .fold(Matrix1xX::<Float>::zeros(dof_robot), |acc, J| acc + J)
            / state.total_mass();

        // Fill in pre-computed Ricatti equation solution S
        let S = DMatrix::from_row_slice(2, 2, &[0.31188605, 0.04863645, 0.04863645, 0.00758452]);
        let B = DMatrix::from_row_slice(2, 1, &[0., 1.]);
        let y_dot_com = (&J_dot_com * &v_full)[(0, 0)];
        let x = vector![y_com, y_dot_com];

        let w_v_dot = 0.001;
        let P = ((z_com / GRAVITY).powi(2) * J_com.transpose() * &J_com
            + DMatrix::identity(dof_robot, dof_robot).scale(w_v_dot))
        .scale(2.);
        let mut P_padded = DMatrix::zeros(dof, dof);
        P_padded
            .view_mut((0, 0), (dof_robot, dof_robot))
            .copy_from(&P);
        let P_padded = CscMatrix::from(P_padded.row_iter());

        let opt_q = ((z_com / GRAVITY).powi(2) * 2. * (&J_dot_com * &v_full)[(0, 0)]
            - 2. * z_com / GRAVITY * y_com
            + 2. * x.tr_mul(&(S * B))[(0, 0)])
            * J_com.transpose()
            - 2. * w_v_dot * v_dot_des;
        let mut opt_q_padded = DVector::zeros(dof);
        opt_q_padded
            .view_mut((0, 0), (dof_robot, 1))
            .copy_from(&opt_q);

        // No-slip constraint on base(foot) body.
        // TODO: constrain only planar movement
        let A_no_slip = DMatrix::<Float>::identity(6, dof);
        let b_no_slip = Vector6::zeros();
        let cones_no_slip = ZeroConeT(6);

        // Add free-body dynamics constraint
        let bodies_to_root = state.get_bodies_to_root_no_update();
        let H: DMatrix<Float> = mass_matrix(state, &bodies_to_root);

        let joint_twists = compute_joint_twists(state);
        let twists = state.get_body_twists();
        let C = dynamics_bias(
            state,
            &bodies_to_root,
            &joint_twists,
            &twists,
            &HashMap::new(),
        );

        // Matrix that transforms contact force to generalized forces
        let mut phi_free = DMatrix::zeros(6, dof_contact);
        let foot_to_root = &bodies_to_root[0];
        let w_foot = 0.2;
        let h_foot = 0.05;
        let cp1 =
            foot_to_root.trans() + foot_to_root.rot() * vector![w_foot / 2., l / 2., -h_foot / 2.];
        let cp2 =
            foot_to_root.trans() + foot_to_root.rot() * vector![-w_foot / 2., l / 2., -h_foot / 2.];
        let cp3 =
            foot_to_root.trans() + foot_to_root.rot() * vector![w_foot / 2., -l / 2., -h_foot / 2.];
        let cp4 = foot_to_root.trans()
            + foot_to_root.rot() * vector![-w_foot / 2., -l / 2., -h_foot / 2.];
        phi_free
            .view_mut((0, 0), (3, 3))
            .copy_from(&skew_symmetric(&cp1));
        phi_free
            .view_mut((3, 0), (3, 3))
            .copy_from(&DMatrix::identity(3, 3));

        phi_free
            .view_mut((0, 3), (3, 3))
            .copy_from(&skew_symmetric(&cp2));
        phi_free
            .view_mut((3, 3), (3, 3))
            .copy_from(&DMatrix::identity(3, 3));

        phi_free
            .view_mut((0, 6), (3, 3))
            .copy_from(&skew_symmetric(&cp3));
        phi_free
            .view_mut((3, 6), (3, 3))
            .copy_from(&DMatrix::identity(3, 3));

        phi_free
            .view_mut((0, 9), (3, 3))
            .copy_from(&skew_symmetric(&cp4));
        phi_free
            .view_mut((3, 9), (3, 3))
            .copy_from(&DMatrix::identity(3, 3));

        phi_free = J_spatial_vs[0].tr_mul(&phi_free);

        let mut A_free_dynamics = DMatrix::<Float>::zeros(6, dof);
        A_free_dynamics
            .view_mut((0, 0), (6, dof_robot))
            .copy_from(&H.rows(0, 6));
        A_free_dynamics
            .view_mut((0, dof_robot), (6, dof_contact))
            .copy_from(&-phi_free.rows(0, 6)); // Note: do not forget negation
        let b_free_dynamics = Vector6::from_row_slice(&(-&C).as_slice()[0..6]); // Note: do not forget negation
        let cone_free_dynamics = ZeroConeT(6);

        // Add friction cone constraint
        let mut A_friction_cone = DMatrix::zeros(dof_contact, dof);
        let mu = 0.95;
        A_friction_cone[(0, dof_robot + 2)] = -mu;
        A_friction_cone[(1, dof_robot)] = -1.;
        A_friction_cone[(2, dof_robot + 1)] = -1.;

        A_friction_cone[(3, dof_robot + 5)] = -mu;
        A_friction_cone[(4, dof_robot + 3)] = -1.;
        A_friction_cone[(5, dof_robot + 4)] = -1.;

        A_friction_cone[(6, dof_robot + 8)] = -mu;
        A_friction_cone[(7, dof_robot + 6)] = -1.;
        A_friction_cone[(8, dof_robot + 7)] = -1.;

        A_friction_cone[(9, dof_robot + 11)] = -mu;
        A_friction_cone[(10, dof_robot + 9)] = -1.;
        A_friction_cone[(11, dof_robot + 10)] = -1.;

        let b_friction_cone = DVector::<Float>::zeros(dof_contact);
        let cones_friction_cone = SecondOrderConeT(3);

        let A = CscMatrix::from(
            A_no_slip
                .row_iter()
                .chain(A_free_dynamics.row_iter())
                .chain(A_friction_cone.row_iter()),
        );
        let b = DVector::from_iterator(
            6 + 6 + dof_contact,
            b_no_slip
                .iter()
                .chain(b_free_dynamics.iter())
                .chain(b_friction_cone.iter())
                .cloned(),
        );
        let cone = [
            cones_no_slip,
            cone_free_dynamics,
            cones_friction_cone.clone(),
            cones_friction_cone.clone(),
            cones_friction_cone.clone(),
            cones_friction_cone,
        ];

        let settings = DefaultSettings::default();
        let mut solver = DefaultSolver::new(
            &P_padded,
            &opt_q_padded.as_slice(),
            &A,
            &b.as_slice(),
            &cone,
            settings,
        );

        solver.solve();
        let sol = solver.solution.x;

        // Inverse-dynamics to compute torque
        let v_dot = DVector::from_row_slice(&sol[0..dof_robot]);
        let H_a: DMatrix<Float> = H.rows(6, dof_actuated).into_owned();
        let C_a: DVector<Float> = C.rows(6, dof_actuated).into_owned();
        let tau = H_a * &v_dot + C_a;

        flog!("tau: {:?}", tau);
        // flog!("tau diff: {}", DVector::from_column_slice(&tau) - v_dot_des);
        flog!("angle diff: {}", q_actuated - q_actuated_des);
        // let tau = q_ddot_des.clone();

        let contact_forces = Matrix4x3::from_row_slice(&sol[dof_robot..]);
        flog!("contact forces: {}", contact_forces);

        let u = J_dot_com * v_full + J_com * &v_dot;
        let y_zmp = y_com - z_com / GRAVITY * u[(0, 0)];
        flog!("y_zmp: {}", y_zmp);

        let joint_torques: Vec<JointTorque> = tau.iter().map(|x| JointTorque::Float(*x)).collect();
        [
            vec![JointTorque::Spatial(SpatialVector::zero())],
            joint_torques,
        ]
        .concat()
    }
}

pub struct LegController {}

impl Controller for LegController {
    fn control(
        &mut self,
        state: &mut crate::mechanism::MechanismState,
        input: Option<&super::ControlInput>,
    ) -> Vec<JointTorque> {
        let q_full = state.q.to_float_dvec();
        let q_actuated: DVector<Float> =
            DVector::from_iterator(q_full.len() - 7, q_full.iter().skip(7).cloned());

        let v_full = state.v.to_float_dvec();
        let dof_robot = v_full.len();
        let dof_unactuated = 6;
        let dof_actuated = dof_robot - dof_unactuated;
        let v_actuated =
            DVector::from_iterator(dof_actuated, v_full.iter().skip(dof_unactuated).cloned());

        let q_actuated_des = vector![-PI / 4., PI / 2., -PI / 4.];
        let v_dot_des_actuated = (q_actuated_des - &q_actuated) - 1. * v_actuated;
        let mut v_dot_des = DVector::zeros(dof_robot);
        v_dot_des
            .view_mut((dof_unactuated, 0), (dof_actuated, 1))
            .copy_from(&v_dot_des_actuated);

        let l = 0.2;
        let z_com = 0.09428090415820636;

        let com = state.center_of_mass();
        let y_com = com.y;

        let n_contacts = 4;
        let dof_contact = n_contacts * 3;
        let dof = dof_robot + dof_contact;

        // Compute J_com systematically
        let J_spatial_vs = state.spatial_velocity_jacobians();
        let mat_linear_v_com = state.com_linear_velocity_extraction_matrices();

        let n_bodies = state.bodies.len();
        let joint_dofs = state.joint_dofs();
        // Jacobian of center-of-mass of each body
        let J_coms: Vec<Matrix1xX<Float>> = (0..n_bodies)
            .map(|i| {
                let mut J_com: Matrix1xX<Float> = Matrix1xX::zeros(dof_robot);
                let mut col_offset = 0;
                for j in 0..=i {
                    let dof = joint_dofs[j];
                    J_com.view_mut((0, col_offset), (1, dof)).copy_from(
                        &(Vector3::y().transpose() * &mat_linear_v_com[i] * &J_spatial_vs[j]),
                    );
                    col_offset += dof;
                }
                J_com
            })
            .collect();
        // Jacobian of center-of-mass of the whole system
        let J_com = izip!(state.bodies.iter(), J_coms.iter())
            .map(|(b, J)| b.inertia.mass * J)
            .fold(Matrix1xX::<Float>::zeros(dof_robot), |acc, J| acc + J)
            / state.total_mass();

        // Compute J_dot_com systematically
        let J_spatial_v_derivs = state.spatial_velocity_jacobian_derivatives();
        let mat_linear_v_com_derivs = state.com_linear_velocity_extraction_matrix_derivatives();

        // time derivative of Jacobian of center-of-mass of each body
        let J_dot_coms: Vec<Matrix1xX<Float>> = (0..n_bodies)
            .map(|i| {
                let mut J_dot_com: Matrix1xX<Float> = Matrix1xX::zeros(dof_robot);
                let mut col_offset = 0;
                for j in 0..=i {
                    let dof = joint_dofs[j];
                    J_dot_com.view_mut((0, col_offset), (1, dof)).copy_from(
                        &(Vector3::<Float>::y().transpose()
                            * (&mat_linear_v_com_derivs[i] * &J_spatial_vs[j]
                                + &mat_linear_v_com[i] * &J_spatial_v_derivs[j])),
                    );
                    col_offset += dof;
                }
                J_dot_com
            })
            .collect();
        // time derivative of Jacobian of center-of-mass of the whole system
        let J_dot_com = izip!(state.bodies.iter(), J_dot_coms.iter())
            .map(|(b, J)| b.inertia.mass * J)
            .fold(Matrix1xX::<Float>::zeros(dof_robot), |acc, J| acc + J)
            / state.total_mass();

        // Fill in pre-computed Ricatti equation solution S
        let S = DMatrix::from_row_slice(2, 2, &[0.19606829, 0.01922139, 0.01922139, 0.00188435]);
        let B = DMatrix::from_row_slice(2, 1, &[0., 1.]);
        let y_dot_com = (&J_dot_com * &v_full)[(0, 0)];
        let x = vector![y_com, y_dot_com];

        let mut P_q_des: DMatrix<Float> = DMatrix::zeros(dof_robot, dof_robot);
        P_q_des
            .view_mut((6, 6), (dof_actuated, dof_actuated))
            .copy_from(&DMatrix::identity(dof_actuated, dof_actuated));
        let w_v_dot = 0.001;
        let P = ((z_com / GRAVITY).powi(2) * J_com.transpose() * &J_com + P_q_des.scale(w_v_dot))
            .scale(2.);
        let mut P_padded = DMatrix::zeros(dof, dof);
        P_padded
            .view_mut((0, 0), (dof_robot, dof_robot))
            .copy_from(&P);
        let P_padded = CscMatrix::from(P_padded.row_iter());

        let opt_q = ((z_com / GRAVITY).powi(2) * 2. * (&J_dot_com * &v_full)[(0, 0)]
            - 2. * z_com / GRAVITY * y_com
            + 2. * x.tr_mul(&(S * B))[(0, 0)])
            * J_com.transpose()
            - 2. * w_v_dot * v_dot_des;
        let mut opt_q_padded = DVector::zeros(dof);
        opt_q_padded
            .view_mut((0, 0), (dof_robot, 1))
            .copy_from(&opt_q);

        // No-slip constraint on foot body.
        // TODO: constrain only planar movement
        let mut J_foot: Matrix6xX<Float> = Matrix6xX::zeros(dof);
        let mut col_offset = 0;
        for i in 0..n_bodies {
            let joint_dof = joint_dofs[i];
            J_foot
                .view_mut((0, col_offset), (6, joint_dof))
                .copy_from(&J_spatial_vs[i]);
            col_offset += joint_dof;
        }

        let mut J_dot_foot: Matrix6xX<Float> = Matrix6xX::zeros(dof_robot);
        let mut col_offset = 0;
        for i in 0..n_bodies {
            let joint_dof = joint_dofs[i];
            J_dot_foot
                .view_mut((0, col_offset), (6, joint_dof))
                .copy_from(&J_spatial_v_derivs[i]);
            col_offset += joint_dof;
        }

        let A_no_slip: DMatrix<Float> =
            DMatrix::from_iterator(J_foot.nrows(), J_foot.ncols(), J_foot.iter().cloned());
        let b_no_slip = -J_dot_foot * &v_full;
        let cones_no_slip = ZeroConeT(6);

        // Add free-body dynamics constraint
        let bodies_to_root = state.get_bodies_to_root_no_update();
        let H: DMatrix<Float> = mass_matrix(state, &bodies_to_root);

        let joint_twists = compute_joint_twists(state);
        let twists = state.get_body_twists();
        let C = dynamics_bias(
            state,
            &bodies_to_root,
            &joint_twists,
            &twists,
            &HashMap::new(),
        );

        // Matrix that transforms contact force to wrench expressed in world frame
        let mut wrench_transform = DMatrix::zeros(6, dof_contact);
        let foot_to_root = &bodies_to_root[0];
        let w_foot = 0.1;
        let h_foot = 0.05;
        let cp1 =
            foot_to_root.trans() + foot_to_root.rot() * vector![w_foot / 2., l / 2., -h_foot / 2.];
        let cp2 =
            foot_to_root.trans() + foot_to_root.rot() * vector![-w_foot / 2., l / 2., -h_foot / 2.];
        let cp3 =
            foot_to_root.trans() + foot_to_root.rot() * vector![w_foot / 2., -l / 2., -h_foot / 2.];
        let cp4 = foot_to_root.trans()
            + foot_to_root.rot() * vector![-w_foot / 2., -l / 2., -h_foot / 2.];
        wrench_transform
            .view_mut((0, 0), (3, 3))
            .copy_from(&skew_symmetric(&cp1));
        wrench_transform
            .view_mut((3, 0), (3, 3))
            .copy_from(&DMatrix::identity(3, 3));

        wrench_transform
            .view_mut((0, 3), (3, 3))
            .copy_from(&skew_symmetric(&cp2));
        wrench_transform
            .view_mut((3, 3), (3, 3))
            .copy_from(&DMatrix::identity(3, 3));

        wrench_transform
            .view_mut((0, 6), (3, 3))
            .copy_from(&skew_symmetric(&cp3));
        wrench_transform
            .view_mut((3, 6), (3, 3))
            .copy_from(&DMatrix::identity(3, 3));

        wrench_transform
            .view_mut((0, 9), (3, 3))
            .copy_from(&skew_symmetric(&cp4));
        wrench_transform
            .view_mut((3, 9), (3, 3))
            .copy_from(&DMatrix::identity(3, 3));

        // Matrix that transforms contact force to generalized forces
        let mut phi = DMatrix::zeros(dof_robot, dof_contact);
        let mut row_offset = 0;
        for i in 0..n_bodies {
            let joint_dof = joint_dofs[i];
            phi.view_mut((row_offset, 0), (joint_dof, dof_contact))
                .copy_from(&J_spatial_vs[i].tr_mul(&wrench_transform));
            row_offset += joint_dof;
        }

        let mut A_free_dynamics = DMatrix::<Float>::zeros(6, dof);
        A_free_dynamics
            .view_mut((0, 0), (6, dof_robot))
            .copy_from(&H.rows(0, 6));
        A_free_dynamics
            .view_mut((0, dof_robot), (6, dof_contact))
            .copy_from(&-phi.rows(0, 6)); // Note: do not forget negation
        let b_free_dynamics = Vector6::from_row_slice(&(-&C).as_slice()[0..6]); // Note: do not forget negation
        let cone_free_dynamics = ZeroConeT(6);

        // Add friction cone constraint
        let mut A_friction_cone = DMatrix::zeros(dof_contact, dof);
        let mu = 0.95;
        A_friction_cone[(0, dof_robot + 2)] = -mu;
        A_friction_cone[(1, dof_robot)] = -1.;
        A_friction_cone[(2, dof_robot + 1)] = -1.;

        A_friction_cone[(3, dof_robot + 5)] = -mu;
        A_friction_cone[(4, dof_robot + 3)] = -1.;
        A_friction_cone[(5, dof_robot + 4)] = -1.;

        A_friction_cone[(6, dof_robot + 8)] = -mu;
        A_friction_cone[(7, dof_robot + 6)] = -1.;
        A_friction_cone[(8, dof_robot + 7)] = -1.;

        A_friction_cone[(9, dof_robot + 11)] = -mu;
        A_friction_cone[(10, dof_robot + 9)] = -1.;
        A_friction_cone[(11, dof_robot + 10)] = -1.;

        let b_friction_cone = DVector::<Float>::zeros(dof_contact);
        let cones_friction_cone = SecondOrderConeT(3);

        let A = CscMatrix::from(
            A_no_slip
                .row_iter()
                .chain(A_free_dynamics.row_iter())
                .chain(A_friction_cone.row_iter()),
        );
        let b = DVector::from_iterator(
            6 + 6 + dof_contact,
            b_no_slip
                .iter()
                .chain(b_free_dynamics.iter())
                .chain(b_friction_cone.iter())
                .cloned(),
        );
        let cone = [
            cones_no_slip,
            cone_free_dynamics,
            cones_friction_cone.clone(),
            cones_friction_cone.clone(),
            cones_friction_cone.clone(),
            cones_friction_cone,
        ];

        let settings = DefaultSettings::default();
        let mut solver = DefaultSolver::new(
            &P_padded,
            &opt_q_padded.as_slice(),
            &A,
            &b.as_slice(),
            &cone,
            settings,
        );

        solver.solve();
        let sol = solver.solution.x;

        let contact_forces = Matrix4x3::from_row_slice(&sol[dof_robot..]);
        flog!("contact forces: {}", contact_forces);

        // Inverse-dynamics to compute torque
        let v_dot = DVector::from_row_slice(&sol[0..dof_robot]);
        let H_a: DMatrix<Float> = H.rows(6, dof_actuated).into_owned();
        let C_a: DVector<Float> = C.rows(6, dof_actuated).into_owned();
        let lambda = DVector::from_column_slice(&sol[dof_robot..]);
        let tau = H_a * &v_dot + C_a - phi.rows(6, dof_actuated) * lambda;

        flog!("tau: {:?}", tau);
        // flog!("tau diff: {}", DVector::from_column_slice(&tau) - v_dot_des);
        flog!("angle diff: {}", q_actuated - q_actuated_des);
        // let tau = q_ddot_des.clone();

        let u = J_dot_com * v_full + J_com * &v_dot;
        let y_zmp = y_com - z_com / GRAVITY * u[(0, 0)];
        flog!("y_zmp: {}", y_zmp);

        let joint_torques: Vec<JointTorque> = tau.iter().map(|x| JointTorque::Float(*x)).collect();

        let force = input.unwrap().floats[2];
        let f = bodies_to_root[0].inv().rot() * vector![0., force * 0.1, 0.];
        [
            vec![JointTorque::Spatial(SpatialVector {
                angular: zero(),
                linear: f,
            })],
            joint_torques,
        ]
        .concat()
    }
}

#[cfg(test)]
mod leg_control_tests {
    use na::{vector, zero, UnitQuaternion, Vector3};

    use crate::{
        builders::leg_builder::build_leg,
        contact::HalfSpace,
        control::{leg_control::LegController, Controller},
        joint::JointPosition,
        simulate::step,
        spatial::pose::Pose,
        PI,
    };

    #[test]
    fn leg_controller_test() {
        // Arrange
        let mut state = build_leg();

        state.add_halfspace(HalfSpace::new(Vector3::z_axis(), -0.05 / 2.));

        let thigh_angle = -PI / 4.;
        let calf_angle = PI / 2.;
        let foot_angle = -PI / 4.;
        let q_init = vec![
            JointPosition::Pose(Pose {
                rotation: UnitQuaternion::from_axis_angle(&Vector3::x_axis(), thigh_angle),
                translation: zero(),
            }),
            JointPosition::Float(calf_angle),
            JointPosition::Float(foot_angle),
        ];

        state.update_q(&q_init);

        // Set the height so that foot touches ground
        let foot_height = state.poses().last().unwrap().translation.z;
        state.set_joint_q(
            1,
            JointPosition::Pose(Pose {
                rotation: UnitQuaternion::from_axis_angle(&Vector3::x_axis(), thigh_angle),
                translation: vector![0., 0., -foot_height],
            }),
        );

        let mut controller = LegController {};

        // Act
        let final_time = 1.0;
        let dt = 1. / 60.;
        let num_steps = (final_time / dt) as usize;
        for _ in 0..num_steps {
            let torque = controller.control(&mut state, None);
            let (_q, _v) = step(
                &mut state,
                dt,
                &torque,
                &crate::integrators::Integrator::VelocityStepping,
            );
        }
    }
}
