use clarabel::{
    algebra::{CscMatrix, MatrixMathMut},
    solver::{
        DefaultSolver, IPSolver,
        SupportedConeT::{self, SecondOrderConeT},
    },
};
use itertools::izip;
use na::{DMatrix, DVector, Matrix1xX, Matrix3, Matrix3xX};

use crate::{
    hybrid::{visual::Visual, Hybrid},
    types::Float,
    util::{
        dual_friction_cone_multipler, friction_cone_multipler,
        spatial_to_linear_velocity_multiplier,
    },
};

impl Hybrid {
    /// Experimental: Step the simulation by dual method, i.e. solving for contact impulses
    pub fn step_dual(&mut self, dt: Float, input: &Vec<Float>) {
        assert_eq!(self.deformables.len(), 0);
        assert_eq!(self.cloths.len(), 0);
        assert_eq!(self.static_bodies.len(), 0);

        // update jacobians and mass_matrix before-hand as cached result
        for articulated in self.articulated.iter_mut() {
            articulated.update_jacobians();
            articulated.update_mass_matrix();
        }

        // Compute controller torques
        let taus: Vec<DVector<Float>> = izip!(self.controllers.iter_mut(), self.articulated.iter())
            .map(|(c, a)| c.control(a, input))
            .collect();

        // Step the controllers for time effect
        for (controller, articulated) in izip!(self.controllers.iter_mut(), self.articulated.iter())
        {
            controller.step(dt, articulated);
        }

        // First, compute free velocities, i.e. v_star
        let v_articulated: Vec<DVector<Float>> =
            izip!(self.articulated.iter_mut(), taus.into_iter())
                .map(|(a, t)| a.free_velocity(dt, t, self.gravity_enabled))
                .collect();
        let total_len = v_articulated.iter().map(|v| v.len()).sum::<usize>();
        let v_star: DVector<Float> = DVector::from_iterator(
            total_len,
            v_articulated.iter().flat_map(|v| v.data.as_vec().clone()),
        );

        let offset_articulated = 0; // starting offset for articulated velocity within stacked total velocity vector
        let dof_articulated: usize = self.articulated.iter().map(|a| a.dof()).sum();
        let total_dof = offset_articulated + dof_articulated;

        // contact handling
        // reference: Contact Models in Robotics, 2024
        //            equation (20) - Optimization on the dual
        let mut Js: Vec<Matrix3xX<Float>> = vec![];
        let mu = self.friction_mu; // friction coefficient
        let mut As: Vec<Matrix3<Float>> = vec![];

        // halfspace - articulated collision detection
        for halfspace in self.halfspaces.iter() {
            let n = &halfspace.normal;
            let mut icol_arti = offset_articulated;
            for (_i_articulated, articulated) in self.articulated.iter().enumerate() {
                let dof = articulated.dof();
                for (i_joint, (rigid, _joint)) in
                    izip!(articulated.bodies.iter(), articulated.joints.iter()).enumerate()
                {
                    let mut cp_normal_list = vec![];
                    for (collider, iso_collider_to_body, _color, _mu) in rigid.visual.iter() {
                        let iso = rigid.pose.to_isometry() * iso_collider_to_body;
                        let collider_pos = iso.translation.vector;

                        match collider {
                            Visual::Point(_point) => {
                                if halfspace.has_inside(&collider_pos) {
                                    cp_normal_list.push((collider_pos, n));
                                }
                            }
                            Visual::Sphere(sphere) => {
                                if let Some(cp) =
                                    halfspace.intersect_sphere(&collider_pos, sphere.r)
                                {
                                    cp_normal_list.push((cp, n));
                                }
                            }
                            Visual::Cuboid(cuboid) => {
                                for point in cuboid.points(&iso) {
                                    if halfspace.has_inside(&point) {
                                        cp_normal_list.push((point, n));
                                    }
                                }
                            }

                            Visual::RigidMesh(_mesh) => {
                                // println!("ignore collision detection between halfspace and articulated rigid mesh");
                            }
                        }
                    }

                    // Assemble all the contact constraint joint jacobians for this body
                    for (cp, n) in cp_normal_list.iter() {
                        let C = friction_cone_multipler(&n, mu);
                        As.push(-C);

                        let mut J = Matrix3xX::zeros(total_dof);
                        let H = articulated.total_jacobian_to_body(i_joint);
                        let X = spatial_to_linear_velocity_multiplier(&cp);
                        J.view_mut((0, icol_arti), (3, dof)).copy_from(&(X * H));

                        Js.push(J);
                    }
                }
                icol_arti += dof;
            }
        }

        // Mass matrix needs to be computed per step because it depends on the current joint q
        let mut M: DMatrix<Float> = DMatrix::zeros(total_dof, total_dof);

        // Assemble M for articulated bodies
        let mut i = offset_articulated;
        for articulated in self.articulated.iter() {
            let dof = articulated.dof();
            M.view_mut((i, i), (dof, dof))
                .copy_from(&articulated.mass_matrix);
            i += dof;
        }

        // Compute overall contact Jacobian
        let J = if Js.len() > 0 {
            let mut rows: Vec<Matrix1xX<Float>> = vec![];
            for contact_J in Js.iter() {
                rows.extend(contact_J.row_iter().map(|r| r.into_owned()));
            }
            let J = DMatrix::from_rows(&rows);
            J
        } else {
            DMatrix::zeros(0, total_dof)
        };

        let M_inverse = M.cholesky().unwrap().inverse();
        let M_inverse_J_transpose = M_inverse * J.transpose();
        let G = &J * &M_inverse_J_transpose;
        let g = J * &v_star;

        // Formulate and solve the second order cone program
        let P = CscMatrix::from(G.row_iter());
        let q: Vec<Float> = Vec::from(g.as_slice());

        // let A = CscMatrix::<Float>::identity(g.nrows());
        let A = blocks_to_block_diagonal(&As);
        let b = vec![0.; g.nrows()];
        let cones: Vec<SupportedConeT<Float>> = vec![SecondOrderConeT(3); Js.len()];

        self.solver =
            DefaultSolver::new(&P, &q, &A, &b, &cones, self.solver.settings().clone()).unwrap();

        let lambda_sol = if Js.len() > 0 {
            self.solver.solve();
            let sol = DVector::from(self.solver.solution.x.clone());
            sol
        } else {
            DVector::zeros(0)
        };

        let v_sol = v_star + M_inverse_J_transpose * lambda_sol;

        // Update articulated velocities and poses
        let mut i = offset_articulated;
        for articulated in self.articulated.iter_mut() {
            let dof = articulated.dof();
            let v = v_sol.rows(i, dof).into_owned();
            articulated.integrate(v, dt);
            i += dof;
        }
    }

    /// Solve for each articulated individually, in the hope of speeding up.
    /// However, seems that there is no effect.
    pub fn step_individually(&mut self, dt: Float, input: &Vec<Float>) {
        assert_eq!(self.deformables.len(), 0);
        assert_eq!(self.cloths.len(), 0);
        assert_eq!(self.static_bodies.len(), 0);

        // update jacobians and mass_matrix before-hand as cached result
        for articulated in self.articulated.iter_mut() {
            articulated.update_jacobians();
            articulated.update_mass_matrix();
        }

        // Compute controller torques
        let taus: Vec<DVector<Float>> = izip!(self.controllers.iter_mut(), self.articulated.iter())
            .map(|(c, a)| c.control(a, input))
            .collect();

        // Step the controllers for time effect
        for (controller, articulated) in izip!(self.controllers.iter_mut(), self.articulated.iter())
        {
            controller.step(dt, articulated);
        }

        for (articulated, tau) in izip!(self.articulated.iter_mut(), taus.into_iter()) {
            let v_star: DVector<Float> = articulated.free_velocity(dt, tau, self.gravity_enabled);
            let dof = articulated.dof();

            // Contact handling
            let mut Js: Vec<Matrix3xX<Float>> = vec![];
            let mu = self.friction_mu; // friction coefficient

            // halfspace - articulated collision detection
            for halfspace in self.halfspaces.iter() {
                let n = &halfspace.normal;
                for (i_joint, (rigid, _joint)) in
                    izip!(articulated.bodies.iter(), articulated.joints.iter()).enumerate()
                {
                    let mut cp_normal_list = vec![];
                    for (collider, iso_collider_to_body, _color, _mu) in rigid.visual.iter() {
                        let iso = rigid.pose.to_isometry() * iso_collider_to_body;
                        let collider_pos = iso.translation.vector;

                        match collider {
                            Visual::Point(_point) => {
                                if halfspace.has_inside(&collider_pos) {
                                    cp_normal_list.push((collider_pos, n));
                                }
                            }
                            Visual::Sphere(sphere) => {
                                if let Some(cp) =
                                    halfspace.intersect_sphere(&collider_pos, sphere.r)
                                {
                                    cp_normal_list.push((cp, n));
                                }
                            }
                            Visual::Cuboid(cuboid) => {
                                for point in cuboid.points(&iso) {
                                    if halfspace.has_inside(&point) {
                                        cp_normal_list.push((point, n));
                                    }
                                }
                            }

                            Visual::RigidMesh(_mesh) => {
                                // println!("ignore collision detection between halfspace and articulated rigid mesh");
                            }
                        }
                    }

                    // Assemble all the contact constraint joint jacobians for this body
                    for (cp, n) in cp_normal_list.iter() {
                        let C = dual_friction_cone_multipler(&n, mu);

                        let H = articulated.total_jacobian_to_body(i_joint);
                        let X = spatial_to_linear_velocity_multiplier(&cp);
                        let J: Matrix3xX<Float> = C * X * H;
                        Js.push(J);
                    }
                }
            }

            let M = &articulated.mass_matrix;

            // Solve convex optimization to resolve contact
            let P = CscMatrix::from(M.row_iter());
            let g = -v_star.transpose() * M;
            let q: Vec<Float> = Vec::from(g.as_slice());

            let A = if Js.len() > 0 {
                let mut rows: Vec<Matrix1xX<Float>> = vec![];
                for contact_J in Js.iter() {
                    rows.extend(contact_J.row_iter().map(|r| r.into_owned()));
                }
                let J = DMatrix::from_rows(&rows);

                let mut J = CscMatrix::from(J.row_iter());
                J.scale(-1.);
                J
            } else {
                CscMatrix::zeros((0, dof))
            };
            let b = vec![0.; Js.len() * 3];
            let cones: Vec<SupportedConeT<Float>> = vec![SecondOrderConeT(3); Js.len()];

            self.solver =
                DefaultSolver::new(&P, &q, &A, &b, &cones, self.solver.settings().clone()).unwrap();

            let v_sol = if dof > 0 {
                self.solver.solve();
                DVector::from(self.solver.solution.x.clone())
            } else {
                DVector::zeros(0)
            };

            // Update articulated velocities and poses
            articulated.integrate(v_sol, dt);
        }
    }
}

fn blocks_to_block_diagonal(matrices: &[Matrix3<Float>]) -> CscMatrix<Float> {
    let n = matrices.len();
    let dim = 3 * n;

    // Each 3x3 block has 9 elements. Total non-zeros = 9 * n
    let mut nzval = Vec::with_capacity(9 * n);
    let mut rowval = Vec::with_capacity(9 * n);

    // colptr size is (total_columns + 1)
    let mut colptr = Vec::with_capacity(dim + 1);
    colptr.push(0);

    for (block_idx, mat) in matrices.iter().enumerate() {
        let offset = block_idx * 3;

        // nalgebra matrices are column-major by default.
        // We iterate through columns (j) then rows (i) to match CSC order.
        for j in 0..3 {
            for i in 0..3 {
                let val = mat[(i, j)];

                // Optional: You could add an 'if val != 0.0' check here
                // if you want a truly sparse representation, but for
                // small 3x3 blocks, keeping zeros is often fine.
                nzval.push(val);
                rowval.push(offset + i);
            }
            // colptr tracks the index in nzval where the next column starts
            colptr.push(nzval.len());
        }
    }

    // Convert to Clarabel's internal CscMatrix format
    CscMatrix::new(
        dim, // rows
        dim, // cols
        colptr, rowval, nzval,
    )
}
