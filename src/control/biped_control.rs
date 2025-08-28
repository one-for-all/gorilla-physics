use std::collections::HashMap;

use clarabel::{
    algebra::CscMatrix,
    solver::{
        DefaultSettings, DefaultSettingsBuilder, DefaultSolver, IPSolver,
        SupportedConeT::{SecondOrderConeT, ZeroConeT},
    },
};
use itertools::izip;
use na::{
    vector, zero, DMatrix, DVector, Matrix2x3, Matrix2xX, Matrix4, Matrix4x2, Matrix6xX, Vector3,
    Vector6,
};

use crate::{
    control::Controller,
    dynamics::dynamics_bias,
    flog,
    joint::{JointTorque, ToFloatDVec},
    mechanism::mass_matrix,
    spatial::{spatial_vector::SpatialVector, twist::compute_joint_twists},
    types::Float,
    util::skew_symmetric,
    GRAVITY, PI,
};

pub struct BipedController {}

impl Controller for BipedController {
    fn control(
        &mut self,
        state: &mut crate::mechanism::MechanismState,
        input: Option<&super::ControlInput>,
    ) -> Vec<JointTorque> {
        let q_full = state.q.to_float_dvec();
        let q_actuated = DVector::from_iterator(q_full.len() - 7, q_full.iter().skip(7).cloned());

        let v_full = state.v.to_float_dvec();
        let dof_robot = v_full.len();
        let dof_unactuated = 6;
        let dof_actuated = dof_robot - dof_unactuated;
        let v_actuated =
            DVector::from_iterator(dof_actuated, v_full.iter().skip(dof_unactuated).cloned());

        let q_actuated_des = vector![
            0.,
            0.,
            -PI / 4.,
            PI / 2.,
            -PI / 4., // ankle
            0.,       // foot
            0.,
            0.,
            -PI / 4.,
            PI / 2.,
            -PI / 4.,
            0.,
        ];
        let v_dot_des_actuated = (q_actuated_des - &q_actuated) - 1. * v_actuated;
        let mut v_dot_des = DVector::zeros(dof_robot);
        v_dot_des
            .view_mut((dof_unactuated, 0), (dof_actuated, 1))
            .copy_from(&v_dot_des_actuated);

        let l = 0.2;
        let w_foot = 0.2;
        let h_foot = 0.05;
        let z_com = 0.22537684517864107;

        let com = state.center_of_mass();
        let y_com = com.y;
        let x_com = com.x;

        let n_contacts = 8;
        let dof_contact = n_contacts * 3;
        let dof = dof_robot + dof_contact;

        // Compute J_com systematically
        let J_spatial_vs = state.spatial_velocity_jacobians();
        let mat_linear_v_com = state.com_linear_velocity_extraction_matrices();

        let n_bodies = state.bodies.len();
        let joint_dofs = state.joint_dofs();
        #[rustfmt::skip]
         let xy_extraction = Matrix2x3::new(
             1., 0., 0.,
             0., 1., 0.
         );

        // Column offset of each body/joint
        let mut dof_offsets = vec![];
        let mut cur_offset = 0;
        for i in 0..n_bodies {
            dof_offsets.push(cur_offset);
            let joint_dof = joint_dofs[i];
            cur_offset += joint_dof;
        }

        // Jacobian of center-of-mass of each body
        let mut J_coms = vec![Matrix2xX::<Float>::zeros(dof_robot); n_bodies];
        for i in 0..n_bodies {
            let jointid = i + 1;
            let mut currentid = jointid;
            while currentid != 0 {
                let j = currentid - 1;
                let joint_dof = joint_dofs[j];
                let col_offset = dof_offsets[j];
                J_coms[i]
                    .view_mut((0, col_offset), (2, joint_dof))
                    .copy_from(&(xy_extraction * &mat_linear_v_com[i] * &J_spatial_vs[j]));

                currentid = state.parents[currentid - 1];
            }
        }
        // Jacobian of center-of-mass of the whole system
        let J_com = izip!(state.bodies.iter(), J_coms.iter())
            .map(|(b, J)| b.inertia.mass * J)
            .fold(Matrix2xX::<Float>::zeros(dof_robot), |acc, J| acc + J)
            / state.total_mass();

        // Compute J_dot_com systematically
        let J_spatial_v_derivs = state.spatial_velocity_jacobian_derivatives();
        let mat_linear_v_com_derivs = state.com_linear_velocity_extraction_matrix_derivatives();

        // time derivative of Jacobian of center-of-mass of each body
        let mut J_dot_coms = vec![Matrix2xX::<Float>::zeros(dof_robot); n_bodies];
        for i in 0..n_bodies {
            let jointid = i + 1;
            let mut currentid = jointid;
            while currentid != 0 {
                let j = currentid - 1;
                let joint_dof = joint_dofs[j];
                let col_offset = dof_offsets[j];
                J_dot_coms[i]
                    .view_mut((0, col_offset), (2, joint_dof))
                    .copy_from(
                        &(xy_extraction
                            * (&mat_linear_v_com_derivs[i] * &J_spatial_vs[j]
                                + &mat_linear_v_com[i] * &J_spatial_v_derivs[j])),
                    );

                currentid = state.parents[currentid - 1];
            }
        }
        // time derivative of Jacobian of center-of-mass of the whole system
        let J_dot_com = izip!(state.bodies.iter(), J_dot_coms.iter())
            .map(|(b, J)| b.inertia.mass * J)
            .fold(Matrix2xX::<Float>::zeros(dof_robot), |acc, J| acc + J)
            / state.total_mass();

        // Fill in pre-computed Ricatti equation solution S
        #[rustfmt::skip]
        let S = Matrix4::new(
            3.03144812e-01, 5.97128858e-18, 4.59483884e-02, -4.12807977e-20,
            5.97128858e-18, 3.03144812e-01, 7.61635976e-19, 4.59483884e-02,
            4.59483884e-02, 7.61635976e-19, 6.96450778e-03, -4.24811204e-21,
            -4.12807977e-20, 4.59483884e-02, -4.24811204e-21, 6.96450778e-03,
        );
        #[rustfmt::skip]
        let B = Matrix4x2::new(
            0., 0.,
            0., 0.,
            1., 0.,
            0., 1.,
        );

        let com_dot = &J_dot_com * &v_full;
        let x = vector![x_com, y_com, com_dot[0], com_dot[1]];

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

        let opt_q = (((z_com / GRAVITY).powi(2) * 2. * (&J_dot_com * &v_full).transpose()
            - 2. * z_com / GRAVITY * vector![x_com, y_com].transpose()
            + 2. * x.tr_mul(&(S * B)))
            * &J_com)
            .transpose()
            - 2. * w_v_dot * v_dot_des;
        let mut opt_q_padded = DVector::zeros(dof);
        opt_q_padded
            .view_mut((0, 0), (dof_robot, 1))
            .copy_from(&opt_q);

        // No-slip constraint on left foot.
        // TODO: constrain only planar movement
        let left_foot_id = 7;
        let mut J_left_foot: Matrix6xX<Float> = Matrix6xX::zeros(dof);
        let mut currentid = left_foot_id;
        while currentid != 0 {
            let j = currentid - 1;
            let joint_dof = joint_dofs[j];
            let col_offset = dof_offsets[j];
            J_left_foot
                .view_mut((0, col_offset), (6, joint_dof))
                .copy_from(&J_spatial_vs[j]);

            currentid = state.parents[currentid - 1];
        }

        let mut J_dot_left_foot: Matrix6xX<Float> = Matrix6xX::zeros(dof_robot);
        let mut currentid = left_foot_id;
        while currentid != 0 {
            let j = currentid - 1;
            let joint_dof = joint_dofs[j];
            let col_offset = dof_offsets[j];
            J_dot_left_foot
                .view_mut((0, col_offset), (6, joint_dof))
                .copy_from(&J_spatial_v_derivs[j]);

            currentid = state.parents[currentid - 1];
        }

        let A_left_no_slip: DMatrix<Float> = DMatrix::from_iterator(
            J_left_foot.nrows(),
            J_left_foot.ncols(),
            J_left_foot.iter().cloned(),
        );
        let b_left_no_slip = -J_dot_left_foot * &v_full;
        let cones_left_no_slip = ZeroConeT(6);

        // No-slip constraint on right foot.
        // TODO: constrain only planar movement
        let right_foot_id = 13;
        let mut J_right_foot: Matrix6xX<Float> = Matrix6xX::zeros(dof);
        let mut currentid = right_foot_id;
        while currentid != 0 {
            let j = currentid - 1;
            let joint_dof = joint_dofs[j];
            let col_offset = dof_offsets[j];
            J_right_foot
                .view_mut((0, col_offset), (6, joint_dof))
                .copy_from(&J_spatial_vs[j]);

            currentid = state.parents[currentid - 1];
        }

        let mut J_dot_right_foot: Matrix6xX<Float> = Matrix6xX::zeros(dof_robot);
        let mut currentid = right_foot_id;
        while currentid != 0 {
            let j = currentid - 1;
            let joint_dof = joint_dofs[j];
            let col_offset = dof_offsets[j];
            J_dot_right_foot
                .view_mut((0, col_offset), (6, joint_dof))
                .copy_from(&J_spatial_v_derivs[j]);

            currentid = state.parents[currentid - 1];
        }

        let A_right_no_slip: DMatrix<Float> = DMatrix::from_iterator(
            J_right_foot.nrows(),
            J_right_foot.ncols(),
            J_right_foot.iter().cloned(),
        );
        let b_right_no_slip = -J_dot_right_foot * &v_full;
        let cones_right_no_slip = ZeroConeT(6);

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
        let mut left_wrench_transform = DMatrix::zeros(6, dof_contact);
        let left_foot_to_root = &bodies_to_root[left_foot_id];
        let cp1 = left_foot_to_root.trans()
            + left_foot_to_root.rot() * vector![w_foot / 2., l / 2., -h_foot / 2.];
        let cp2 = left_foot_to_root.trans()
            + left_foot_to_root.rot() * vector![-w_foot / 2., l / 2., -h_foot / 2.];
        let cp3 = left_foot_to_root.trans()
            + left_foot_to_root.rot() * vector![w_foot / 2., -l / 2., -h_foot / 2.];
        let cp4 = left_foot_to_root.trans()
            + left_foot_to_root.rot() * vector![-w_foot / 2., -l / 2., -h_foot / 2.];
        left_wrench_transform
            .view_mut((0, 0), (3, 3))
            .copy_from(&skew_symmetric(&cp1));
        left_wrench_transform
            .view_mut((3, 0), (3, 3))
            .copy_from(&DMatrix::identity(3, 3));

        left_wrench_transform
            .view_mut((0, 3), (3, 3))
            .copy_from(&skew_symmetric(&cp2));
        left_wrench_transform
            .view_mut((3, 3), (3, 3))
            .copy_from(&DMatrix::identity(3, 3));

        left_wrench_transform
            .view_mut((0, 6), (3, 3))
            .copy_from(&skew_symmetric(&cp3));
        left_wrench_transform
            .view_mut((3, 6), (3, 3))
            .copy_from(&DMatrix::identity(3, 3));

        left_wrench_transform
            .view_mut((0, 9), (3, 3))
            .copy_from(&skew_symmetric(&cp4));
        left_wrench_transform
            .view_mut((3, 9), (3, 3))
            .copy_from(&DMatrix::identity(3, 3));

        let mut right_wrench_transform = DMatrix::zeros(6, dof_contact);
        let right_foot_to_root = &bodies_to_root[right_foot_id];
        let cp5 = right_foot_to_root.trans()
            + right_foot_to_root.rot() * vector![w_foot / 2., l / 2., -h_foot / 2.];
        let cp6 = right_foot_to_root.trans()
            + right_foot_to_root.rot() * vector![-w_foot / 2., l / 2., -h_foot / 2.];
        let cp7 = right_foot_to_root.trans()
            + right_foot_to_root.rot() * vector![w_foot / 2., -l / 2., -h_foot / 2.];
        let cp8 = right_foot_to_root.trans()
            + right_foot_to_root.rot() * vector![-w_foot / 2., -l / 2., -h_foot / 2.];
        right_wrench_transform
            .view_mut((0, 12), (3, 3))
            .copy_from(&skew_symmetric(&cp5));
        right_wrench_transform
            .view_mut((3, 12), (3, 3))
            .copy_from(&DMatrix::identity(3, 3));

        right_wrench_transform
            .view_mut((0, 15), (3, 3))
            .copy_from(&skew_symmetric(&cp6));
        right_wrench_transform
            .view_mut((3, 15), (3, 3))
            .copy_from(&DMatrix::identity(3, 3));

        right_wrench_transform
            .view_mut((0, 18), (3, 3))
            .copy_from(&skew_symmetric(&cp7));
        right_wrench_transform
            .view_mut((3, 18), (3, 3))
            .copy_from(&DMatrix::identity(3, 3));

        right_wrench_transform
            .view_mut((0, 21), (3, 3))
            .copy_from(&skew_symmetric(&cp8));
        right_wrench_transform
            .view_mut((3, 21), (3, 3))
            .copy_from(&DMatrix::identity(3, 3));

        // Matrix that transforms contact force to generalized forces
        let mut phi = DMatrix::zeros(dof_robot, dof_contact);
        phi.view_mut((dof_offsets[0], 0), (joint_dofs[0], dof_contact))
            .copy_from(
                &J_spatial_vs[0].tr_mul(&(&left_wrench_transform + &right_wrench_transform)),
            );

        // left foot contact wrench
        for i in 1..=left_foot_id - 1 {
            phi.view_mut((dof_offsets[i], 0), (joint_dofs[i], dof_contact))
                .copy_from(&J_spatial_vs[i].tr_mul(&left_wrench_transform));
        }

        // right foot contact wrench
        for i in left_foot_id..=right_foot_id - 1 {
            phi.view_mut((dof_offsets[i], 0), (joint_dofs[i], dof_contact))
                .copy_from(&J_spatial_vs[i].tr_mul(&right_wrench_transform));
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

        A_friction_cone[(12, dof_robot + 14)] = -mu;
        A_friction_cone[(13, dof_robot + 12)] = -1.;
        A_friction_cone[(14, dof_robot + 13)] = -1.;

        A_friction_cone[(15, dof_robot + 17)] = -mu;
        A_friction_cone[(16, dof_robot + 15)] = -1.;
        A_friction_cone[(17, dof_robot + 16)] = -1.;

        A_friction_cone[(18, dof_robot + 20)] = -mu;
        A_friction_cone[(19, dof_robot + 18)] = -1.;
        A_friction_cone[(20, dof_robot + 19)] = -1.;

        A_friction_cone[(21, dof_robot + 23)] = -mu;
        A_friction_cone[(22, dof_robot + 21)] = -1.;
        A_friction_cone[(23, dof_robot + 22)] = -1.;

        let b_friction_cone = DVector::<Float>::zeros(dof_contact);
        let cones_friction_cone = SecondOrderConeT::<Float>(3);

        let A = CscMatrix::from(
            A_left_no_slip
                .row_iter()
                .chain(A_right_no_slip.row_iter())
                .chain(A_free_dynamics.row_iter())
                .chain(A_friction_cone.row_iter()),
        );
        let b = DVector::from_iterator(
            6 + 6 + 6 + dof_contact,
            // 6 + 6 + 6,
            b_left_no_slip
                .iter()
                .chain(b_right_no_slip.iter())
                .chain(b_free_dynamics.iter())
                .chain(b_friction_cone.iter())
                .cloned(),
        );
        let cone = [
            cones_left_no_slip,
            cones_right_no_slip,
            cone_free_dynamics,
            cones_friction_cone.clone(),
            cones_friction_cone.clone(),
            cones_friction_cone.clone(),
            cones_friction_cone.clone(),
            cones_friction_cone.clone(),
            cones_friction_cone.clone(),
            cones_friction_cone.clone(),
            cones_friction_cone,
        ];

        let settings = DefaultSettingsBuilder::default()
            .verbose(false)
            .build()
            .unwrap();
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

        let contact_forces = DMatrix::from_row_slice(n_contacts, 3, &sol[dof_robot..]);
        // flog!("contact forces: {}", contact_forces);

        // Inverse-dynamics to compute torque
        let v_dot = DVector::from_row_slice(&sol[0..dof_robot]);
        let H_a: DMatrix<Float> = H.rows(6, dof_actuated).into_owned();
        let C_a: DVector<Float> = C.rows(6, dof_actuated).into_owned();
        let lambda = DVector::from_column_slice(&sol[dof_robot..]);
        let tau = H_a * &v_dot + C_a - phi.rows(6, dof_actuated) * lambda;

        // flog!("tau: {:?}", tau);
        // flog!("angle diff: {}", q_actuated - q_actuated_des);

        let u = J_dot_com * v_full + J_com * &v_dot;
        let zmp = vector![x_com, y_com] - z_com / GRAVITY * u;
        // flog!("zmp: {}", zmp);

        let joint_torques: Vec<JointTorque> = tau.iter().map(|x| JointTorque::Float(*x)).collect();

        let f = {
            if let Some(input) = input {
                let force_x = input.floats[0] * 0.1;
                let force_y = input.floats[1] * 0.1;
                let force_z = input.floats[2] * 0.1;
                bodies_to_root[1].inv().rot() * vector![force_x, force_y, force_z]
            } else {
                Vector3::zeros()
            }
        };

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
mod biped_control_tests {
    use na::{vector, zero, UnitQuaternion, Vector3};

    use crate::{
        assert_close, assert_vec_close,
        builders::{biped_builder::build_biped, leg_builder::build_leg},
        contact::HalfSpace,
        control::{
            biped_control::BipedController, leg_control::LegController, ControlInput, Controller,
        },
        joint::JointPosition,
        plot::plot,
        simulate::step,
        spatial::pose::Pose,
        PI,
    };

    #[test]
    fn biped_controller_test() {
        // Arrange
        let mut state = build_biped();

        let h_foot = 0.05;
        state.add_halfspace(HalfSpace::new(Vector3::z_axis(), -h_foot / 2.));

        let thigh_angle = -PI / 4.;
        let calf_angle = PI / 2.;
        let ankle_angle = -PI / 4.;
        let foot_angle = 0.;
        let q_init = vec![
            JointPosition::Pose(Pose {
                rotation: UnitQuaternion::identity(),
                translation: zero(),
            }),
            JointPosition::Float(0.),
            JointPosition::Float(0.),
            JointPosition::Float(thigh_angle),
            JointPosition::Float(calf_angle),
            JointPosition::Float(ankle_angle),
            JointPosition::Float(foot_angle),
            JointPosition::Float(0.),
            JointPosition::Float(0.),
            JointPosition::Float(thigh_angle),
            JointPosition::Float(calf_angle),
            JointPosition::Float(ankle_angle),
            JointPosition::Float(foot_angle),
        ];

        state.update_q(&q_init);

        // Set the height so that foot is touching ground
        let foot_pos = state.poses().last().unwrap().translation;
        let foot_height = foot_pos.z;
        state.set_joint_q(
            1,
            JointPosition::Pose(Pose {
                rotation: UnitQuaternion::identity(),
                translation: vector![0., 0., -foot_height],
            }),
        );

        let poses = state.poses();
        let left_foot_index = 6;
        let left_foot_init_pose = poses[left_foot_index];
        let right_foot_index = 12;
        let right_foot_init_pose = poses[right_foot_index];

        let mut controller = BipedController {};

        // Act
        let final_time = 1.0;
        let dt = 1. / 60. / 5.0;
        let num_steps = (final_time / dt) as usize;
        let input = ControlInput::new(vec![1., 1., 1.]);
        for _ in 0..num_steps {
            let torque = controller.control(&mut state, Some(&input));
            let (_q, _v) = step(
                &mut state,
                dt,
                &torque,
                &crate::integrators::Integrator::VelocityStepping,
            );
        }

        // Assert
        let poses = state.poses();
        let left_foot_pose = poses[left_foot_index];
        let right_foot_pose = poses[right_foot_index];
        // TODO(ccd): use CCD so that tolerance can be smaller & timestep can
        // be larger. Currently the feet would sink into the ground.
        assert_vec_close!(
            left_foot_pose.translation,
            left_foot_init_pose.translation,
            1e-3
        );
        assert_vec_close!(
            right_foot_pose.translation,
            right_foot_init_pose.translation,
            1e-3
        );
        let left_angle_diff = left_foot_pose
            .rotation
            .angle_to(&left_foot_init_pose.rotation);
        assert!(left_angle_diff < 2e-3, "{}", left_angle_diff);
        let right_angle_diff = right_foot_pose
            .rotation
            .angle_to(&right_foot_init_pose.rotation);
        assert!(right_angle_diff < 2e-3, "{}", right_angle_diff);
    }
}
