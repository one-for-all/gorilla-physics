use crate::{
    collision::{mesh::mesh_mesh_collision, CollisionDetector},
    contact::contact_dynamics,
    control::energy_control::spring_force,
    inertia::compute_inertias,
    joint::{Joint, JointAcceleration, JointTorque, JointVelocity, ToFloatDVec},
    mechanism::mass_matrix,
    rigid_body::CollisionGeometry,
    spatial::{
        spatial_vector::SpatialVector,
        transform::{compute_bodies_to_root, Transform3D},
        twist::{
            compute_joint_twists, compute_twist_transformation_matrix, compute_twists_wrt_world,
            Twist,
        },
        wrench::{compute_torques, Wrench},
    },
    types::Float,
    util::{mul_inertia, se3_commutator, skew_symmetric},
    GRAVITY,
};
use clarabel::{
    algebra::CscMatrix,
    solver::{
        DefaultSettings, DefaultSettingsBuilder, DefaultSolver, IPSolver,
        SupportedConeT::{self, SecondOrderConeT},
    },
};
use itertools::izip;
use na::{
    dvector, vector, zero, DMatrix, DVector, Dyn, Matrix1xX, Matrix3, Matrix3x6, Matrix3xX,
    Matrix6, Matrix6xX, UnitVector3, LU,
};
use nalgebra::Vector3;
use std::collections::{HashMap, HashSet};

use crate::{mechanism::MechanismState, spatial::spatial_acceleration::SpatialAcceleration};

/// Apply the Newton-Euler equation to compute the wrench to move each body at
/// given acceleration and velocity:
///     f_i = I_i * a_i + v_i \dualcross I_i * v_i
///
/// Reference: Table 5.1 in "Robot Dynamics Algorithms" by Roy Featherstone
pub fn newton_euler(
    state: &MechanismState,
    accels: &HashMap<usize, SpatialAcceleration>,
    bodies_to_root: &HashMap<usize, Transform3D>,
    twists: &HashMap<usize, Twist>,
) -> HashMap<usize, Wrench> {
    // Compute the body inertias wrt. world frame
    let inertias = compute_inertias(state, bodies_to_root);

    // Compute the wrenches at each joint for each body expressed in world frame
    let mut wrenches: HashMap<usize, Wrench> = HashMap::new();
    for jointid in state.treejointids.iter() {
        let bodyid = jointid;
        let I = inertias.get(bodyid).unwrap();
        let T = twists.get(bodyid).unwrap();
        let Tdot = accels.get(bodyid).unwrap();

        if T.frame != Tdot.frame {
            panic!(
                "T frame {}  is not equal to Tdot frame {} !",
                T.frame, Tdot.frame
            );
        }

        if T.body != Tdot.body || T.base != Tdot.base {
            panic!("T and Tdot body/base do not match!");
        }

        let (mut ang, mut lin) = mul_inertia(
            &I.moment,
            &I.cross_part,
            I.mass,
            &Tdot.angular,
            &Tdot.linear,
        );
        let (angular_momentum, linear_momentum) =
            mul_inertia(&I.moment, &I.cross_part, I.mass, &T.angular, &T.linear);

        ang += T.angular.cross(&angular_momentum) + T.linear.cross(&linear_momentum);
        lin += T.angular.cross(&linear_momentum);
        wrenches.insert(
            *bodyid,
            Wrench {
                frame: T.frame.clone(),
                angular: ang,
                linear: lin,
            },
        );
    }
    wrenches
}

/// Compute the coriolis bias acceleration that results from the body velocity
/// and joint velocity, expressed in body frame.
/// Reference: Chapter 5.3 The Recursive Newton-Euler Algorithm in "Robot Dynamics Algorithms" by Roy Featherstone
pub fn coriolis_accel(body_twist: &Twist, joint_twist: &Twist) -> SpatialAcceleration {
    if body_twist.frame != joint_twist.frame {
        panic!(
            "body_twist frame {}  is not equal to joint_twist frame {} !",
            body_twist.frame, joint_twist.frame
        );
    }
    if body_twist.body != joint_twist.body {
        panic!(
            "body_twist body frame {}  is not equal to joint_twist body frame {} !",
            body_twist.body, joint_twist.body
        )
    }
    if body_twist.base != "world" {
        panic!(
            "body_twist base frame {}  is not equal to world frame {} !",
            body_twist.base, "world"
        )
    }

    let (ang, lin) = se3_commutator(
        &body_twist.angular,
        &body_twist.linear,
        &joint_twist.angular,
        &joint_twist.linear,
    );

    SpatialAcceleration {
        body: body_twist.body.clone(),
        base: body_twist.base.clone(),
        frame: body_twist.frame.clone(),
        angular: ang,
        linear: lin,
    }
}

/// Compute the coriolis bias acceleration for all bodies.
/// Reference: Chapter 5.3 The Recursive Newton-Euler Algorithm in "Robot Dynamics Algorithms" by Roy Featherstone
pub fn compute_coriolis_bias_accelerations(
    state: &MechanismState,
    bodies_to_root: &HashMap<usize, Transform3D>,
    twists: &HashMap<usize, Twist>,
    joint_twists: &HashMap<usize, Twist>,
) -> HashMap<usize, SpatialAcceleration> {
    let mut coriolis_bias_accels = HashMap::new();
    let rootid = 0;
    coriolis_bias_accels.insert(
        rootid,
        SpatialAcceleration {
            body: "world".to_string(),
            base: "world".to_string(),
            frame: "world".to_string(),
            angular: zero(),
            linear: zero(),
        },
    );
    for jointid in state.treejointids.iter() {
        let bodyid = jointid;
        let parentid = state.parents[*jointid - 1];

        let body_to_root = bodies_to_root.get(bodyid).unwrap();
        let root_to_body = body_to_root.inv();

        // body twist wrt. world expressed in body frame
        let body_twist = twists.get(bodyid).unwrap().transform(&root_to_body);
        // joint twist expressed in body frame
        let joint_twist = joint_twists.get(bodyid).unwrap();
        let coriolis_accel = coriolis_accel(&body_twist, joint_twist).transform(body_to_root);

        let parent_bias_accel = coriolis_bias_accels.get(&parentid).unwrap();

        // rename the body frame so that it can be added to additionally
        // computed coriolis accel
        let parent_bias_accel = SpatialAcceleration {
            body: coriolis_accel.body.clone(), // rename to current body frame
            base: parent_bias_accel.base.clone(),
            frame: parent_bias_accel.frame.clone(),
            angular: parent_bias_accel.angular,
            linear: parent_bias_accel.linear,
        };
        coriolis_bias_accels.insert(*bodyid, &parent_bias_accel + &coriolis_accel);
        // Note: joint bias is zero for all of our current joint types, since
        // the apparent derivative of motion subspace S is zero.
        // Therefore, we don't add it here.
    }

    coriolis_bias_accels
}

/// Compute the bias acceleration for each body in world frame.
/// Bias acceleration is the acceleration that the body would have under no
/// external force.
///
/// Here, we add the inv gravity acceleration term to simulate gravity.
/// Imagine the whole system is in a elevator accelerating upwards at 9.81 m/s^2.
pub fn bias_accelerations(
    state: &MechanismState,
    bodies_to_root: &HashMap<usize, Transform3D>,
    twists: &HashMap<usize, Twist>,
    joint_twists: &HashMap<usize, Twist>,
) -> HashMap<usize, SpatialAcceleration> {
    let mut bias_accels = HashMap::new();
    let coriolis_bias_accels =
        compute_coriolis_bias_accelerations(state, bodies_to_root, twists, joint_twists);
    for (jointid, joint) in izip!(state.treejointids.iter(), state.treejoints.iter()) {
        let bodyid = jointid;
        let body_name = &joint.transform().from;
        let inv_gravity_accel = SpatialAcceleration {
            body: body_name.clone(),
            base: "world".to_string(),
            frame: "world".to_string(),
            angular: Vector3::zeros(),
            linear: Vector3::new(0.0, 0.0, GRAVITY),
        };

        bias_accels.insert(*bodyid, &inv_gravity_accel + &coriolis_bias_accels[bodyid]);
    }

    bias_accels
}

/// Compute the 'dynamics bias term' and 'contact torque' term , i.e. the
/// combination of terms
///     c(q, v) - τ_c
/// in the unconstrained joint-space equations of motion
///     M(q) vdot + c(q, v) = τ + τ_c
/// given joint configuration vector q, joint velocity vector v, joint
/// acceleration vector vdot, and contact wrenches.
pub fn dynamics_bias(
    state: &MechanismState,
    bodies_to_root: &HashMap<usize, Transform3D>,
    joint_twists: &HashMap<usize, Twist>,
    twists: &HashMap<usize, Twist>,
    contact_wrenches: &HashMap<usize, Wrench>,
) -> DVector<Float> {
    let bias_accels = bias_accelerations(&state, bodies_to_root, &twists, &joint_twists);

    let mut wrenches = newton_euler(&state, &bias_accels, &bodies_to_root, &twists);

    for (bodyid, contact_wrench) in contact_wrenches {
        wrenches.insert(*bodyid, &wrenches[bodyid] - contact_wrench);
    }

    let torques = compute_torques(state, &wrenches, &bodies_to_root);

    torques
}

/// Solves the dynamics equation:
/// M(q) vdot + c(q, v) = τ
pub fn dynamics_solve(
    mass_matrix: &DMatrix<Float>,
    dynamics_bias: &DVector<Float>,
    tau: &DVector<Float>,
) -> DVector<Float> {
    // dynamics bias term
    let c = dynamics_bias;

    if mass_matrix.shape().0 == 0 {
        return dvector![];
    }

    let vdot = mass_matrix.clone().lu().solve(&(tau - c)).expect(&format!(
        r#"Failed to solve for vdot in M(q) vdot + c(q, v) = τ
            where M = {},
                  c = {},
                  τ = {}
        "#,
        mass_matrix, dynamics_bias, tau
    ));
    vdot
}

/// Compute the dynamic quantities necessary for solving both the continuous and
/// discrete version of the dynamics problem
fn dynamics_quantities(
    state: &mut MechanismState,
) -> (
    HashMap<usize, Transform3D>,
    HashMap<usize, Twist>,
    HashMap<usize, Twist>,
    DMatrix<Float>,
) {
    // Compute the body to root frame transform for each body
    let bodies_to_root = compute_bodies_to_root(state);

    // Compute the twist of each joint
    let joint_twists = compute_joint_twists(state);

    // Compute the twist of the each body with respect to the world frame
    let twists = compute_twists_wrt_world(state, &bodies_to_root, &joint_twists);

    let mass_matrix_lower = mass_matrix(state, &bodies_to_root);

    // Convert lower-triangular matrix to full symmetric matrix M
    let mut mass_matrix = mass_matrix_lower.clone();
    for i in 0..mass_matrix_lower.nrows() {
        for j in (i + 1)..mass_matrix_lower.nrows() {
            mass_matrix[(i, j)] = mass_matrix_lower[(j, i)];
        }
    }

    (bodies_to_root, joint_twists, twists, mass_matrix)
}

/// Add the joint forces resulting from springs attached to prismatic joints
fn addPrismaticJointSpringForce(
    state: &MechanismState,
    tau: &Vec<JointTorque>,
) -> Vec<JointTorque> {
    let mut tau = tau.clone();
    for (jointid, joint) in izip!(state.treejointids.iter(), state.treejoints.iter()) {
        if let Joint::PrismaticJoint(joint) = joint {
            if let Some(spring) = &joint.spring {
                let l = *state.q[*jointid - 1].float();

                let f_spring = spring_force(spring.l, l, spring.k); // force in the direction of spring

                tau[*jointid - 1] = JointTorque::Float(tau[*jointid - 1].float() + f_spring);
            }
        }
    }
    tau
}

/// Compute the joint acceleration vector vdot that satisfies the joint-space
/// equations of motion:
///     M(q)vdot + c(q, v) = τ
/// given joint configuration vector q, joint velocity vector v, and joint
/// torques τ.
pub fn dynamics_continuous(
    state: &mut MechanismState,
    tau: &Vec<JointTorque>,
) -> Vec<JointAcceleration> {
    let (bodies_to_root, joint_twists, twists, mass_matrix) = dynamics_quantities(state);

    // Compute the contact wrenches resulting from contacts
    let contact_wrenches = contact_dynamics(state, &bodies_to_root, &twists);

    let dynamics_bias = dynamics_bias(
        state,
        &bodies_to_root,
        &joint_twists,
        &twists,
        &contact_wrenches,
    ); // c(q, v) - τ_contact

    // Spring force from springs attached to prismatic joints
    let tau = addPrismaticJointSpringForce(state, tau);

    let vdot = dynamics_solve(&mass_matrix, &dynamics_bias, &tau.to_float_dvec());

    // Convert from raw floats to joint acceleration types
    let mut i = 0;
    state
        .v
        .iter()
        .map(|v| match v {
            JointVelocity::Float(_) => {
                let vdot = vdot[i];
                i += 1;
                JointAcceleration::Float(vdot)
            }
            JointVelocity::Spatial(_) => {
                let angular = vector![vdot[i], vdot[i + 1], vdot[i + 2]];
                let linear = vector![vdot[i + 3], vdot[i + 4], vdot[i + 5]];
                i += 6;
                JointAcceleration::Spatial(SpatialVector { angular, linear })
            }
            JointVelocity::None => JointAcceleration::None,
        })
        .collect()
}

/// Dynamics method for velocity-stepping method
pub fn dynamics_discrete(
    state: &mut MechanismState,
    tau: &Vec<JointTorque>,
    dt: Float,
) -> Vec<JointVelocity> {
    let (bodies_to_root, joint_twists, twists, mass_matrix) = dynamics_quantities(state);

    let dynamics_bias = dynamics_bias(
        state,
        &bodies_to_root,
        &joint_twists,
        &twists,
        &HashMap::new(), // no explicit contact wrenches in velocity-stepping method
    ); // c(q, v) - τ_contact

    // Spring force from springs attached to prismatic joints
    let tau = addPrismaticJointSpringForce(state, tau);

    let v_current = state.v.to_float_dvec();
    let v_update = dynamics_solve(
        &mass_matrix,
        &(dynamics_bias * dt),
        &(tau.to_float_dvec() * dt),
    );
    let v_free = &v_current + v_update;

    // Construct the building blocks for Jacobian between generalized v and
    // world frame spatial velocity
    let jacobian_blocks: Vec<Matrix6xX<Float>> = build_jacobian_blocks(&state, &bodies_to_root);

    // Build up the Jacobians for the joint constraints
    let constraint_Js: Vec<DMatrix<Float>> =
        build_constraint_jacobians(&state, &jacobian_blocks, &bodies_to_root, v_free.len());

    // contacts is Vec of (contact normal, contact point, bodyid, other_bodyid)
    let contacts = collision_detection(state, &bodies_to_root);

    // Vec of Jacobian rows, which will be assembled into the Jacobian
    let contact_Js: Vec<Matrix3xX<Float>> = contacts
        .iter()
        .map(|(normal, point, bodyid, other_bodyid)| {
            compose_contact_jacobian(
                &state,
                &normal,
                &point,
                *bodyid,
                *other_bodyid,
                v_free.len(),
                &jacobian_blocks,
            )
        })
        .collect();

    let v_new = solve_constraint_and_contact(&constraint_Js, &contact_Js, &v_free, mass_matrix);

    // Convert from raw floats to joint velocity types
    let mut i = 0;
    state
        .v
        .iter()
        .map(|v| match v {
            JointVelocity::Float(_) => {
                let v_new = v_new[i];
                i += 1;
                JointVelocity::Float(v_new)
            }
            JointVelocity::Spatial(_) => {
                let angular = vector![v_new[i], v_new[i + 1], v_new[i + 2]];
                let linear = vector![v_new[i + 3], v_new[i + 4], v_new[i + 5]];
                i += 6;
                JointVelocity::Spatial(SpatialVector { angular, linear })
            }
            JointVelocity::None => JointVelocity::None,
        })
        .collect()
}

/// Build the Jacobian blocks that transform generalized v to
/// world frame spatial velocity.
/// Each joint gives a Matrix6xX, ordered by jointid, except for fixed joint, /// because it has no velocity.
/// Ref: Contact and Friction Simulation for Computer Graphics, 2022,
///      Section 1.5 The Coulomb Friction Law
fn build_jacobian_blocks(
    state: &MechanismState,
    bodies_to_root: &HashMap<usize, Transform3D>,
) -> Vec<Matrix6xX<Float>> {
    izip!(state.treejointids.iter(), state.treejoints.iter())
        .filter_map(|(bodyid, joint)| {
            let body_to_root = bodies_to_root.get(bodyid).unwrap();
            let T = compute_twist_transformation_matrix(&body_to_root.iso);
            match joint {
                Joint::RevoluteJoint(joint) => Some(
                    T * Matrix6xX::from_column_slice(&[
                        joint.axis[0],
                        joint.axis[1],
                        joint.axis[2],
                        0.,
                        0.,
                        0.,
                    ]),
                ),
                Joint::PrismaticJoint(joint) => Some(
                    T * Matrix6xX::from_column_slice(&[
                        0.,
                        0.,
                        0.,
                        joint.axis[0],
                        joint.axis[1],
                        joint.axis[2],
                    ]),
                ),
                Joint::FloatingJoint(_) => Some(Matrix6xX::from_columns(
                    &T.column_iter().collect::<Vec<_>>(),
                )),
                Joint::FixedJoint(_) => None,
            }
        })
        .collect()
}

/// Build, for each constraint, a jacobian that transforms joint velocity to /// constraint frame velocity
fn build_constraint_jacobians(
    state: &MechanismState,
    blocks: &Vec<Matrix6xX<Float>>,
    bodies_to_root: &HashMap<usize, Transform3D>,
    dof: usize, // state v dimension
) -> Vec<DMatrix<Float>> {
    state
        .constraints
        .iter()
        .map(|constraint| {
            let frame1_linked_bodyids = state.linked_bodyids(constraint.frame1());
            let frame2_linked_bodyids = state.linked_bodyids(constraint.frame2());

            let frame1_only_linked_bodyids: HashSet<&usize> = frame1_linked_bodyids
                .difference(&frame2_linked_bodyids)
                .collect();
            let frame2_only_linked_bodyids: HashSet<&usize> = frame2_linked_bodyids
                .difference(&frame1_linked_bodyids)
                .collect();

            let mut J = DMatrix::zeros(6, dof);
            let mut col_offset = 0;
            for (index, block) in blocks.iter().enumerate() {
                // TODO(fixed_joint): take care of this in the presence of fixed joint
                let n_cols = block.ncols();
                let bodyid = index + 1;
                if frame1_only_linked_bodyids.contains(&bodyid) {
                    J.columns_mut(col_offset, n_cols).copy_from(block);
                } else if frame2_only_linked_bodyids.contains(&bodyid) {
                    J.columns_mut(col_offset, n_cols).copy_from(&(-block));
                }
                col_offset += n_cols;
            }

            let frame1_body_to_root = bodies_to_root
                .iter()
                .find(|x| x.1.from == constraint.frame1())
                .unwrap()
                .1;
            let constraint_to_root = frame1_body_to_root.iso * constraint.to_frame1();
            let root_to_constraint = constraint_to_root.inverse();
            let T = compute_twist_transformation_matrix(&root_to_constraint);
            let J = T * J;

            let selection_matrix = constraint.constraint_matrix();
            let J = selection_matrix * J;

            // Filter out rows with all near-zeros
            let (nrows, ncols) = J.shape();
            // Collect indices of non-zero rows
            let non_zero_row_indices: Vec<_> = (0..nrows)
                .filter(|&i| {
                    // Check if any element in the row is not near-zero
                    // Checking near-zero rather than precisely zero,
                    // because rows with only very small values might
                    // turn G = J * mass_inv * J^T into not positive definite.
                    // TODO: better way than this?
                    (0..ncols).any(|j| J[(i, j)].abs() > 1e-4)
                    // (0..ncols).any(|j| J[(i, j)].abs() != 0.0)
                })
                .collect();

            assert!(non_zero_row_indices.len() > 0, "unfiltered J: {}", J);
            DMatrix::from_fn(non_zero_row_indices.len(), ncols, |i, j| {
                J[(non_zero_row_indices[i], j)]
            })
        })
        .collect()
}

/// Performs collision detection and returns a vec of (contact normal, contact point, bodyid, other_bodyid)
fn collision_detection(
    state: &mut MechanismState,
    bodies_to_root: &HashMap<usize, Transform3D>,
) -> Vec<(UnitVector3<Float>, Vector3<Float>, usize, Option<usize>)> {
    let mut contacts: Vec<(UnitVector3<Float>, Vector3<Float>, usize, Option<usize>)> = vec![];

    // Handle point contacts with halfspaces
    for (bodyid, body) in izip!(state.treejointids.iter(), state.bodies.iter()) {
        let body_to_root = bodies_to_root.get(&bodyid).unwrap();

        for contact_point in &body.contact_points {
            let contact_point_world = contact_point.transform(body_to_root); // contact point in world frame
            for halfspace in &state.halfspaces {
                if !contact_point_world.inside_halfspace(&halfspace) {
                    continue;
                }
                contacts.push((
                    halfspace.normal,
                    contact_point_world.location,
                    *bodyid,
                    None,
                ));
            }
        }
    }

    state.update_collidable_mesh_vertex_positions();

    // Handle contacts between body colliders and half-spaces
    for (bodyid, body) in izip!(state.treejointids.iter(), state.bodies.iter()) {
        if let Some(collider) = &body.collider {
            if !collider.enabled {
                continue;
            }
            match &collider.geometry {
                CollisionGeometry::Mesh(mesh) => {
                    for vertex in &mesh.vertices {
                        for halfspace in &state.halfspaces {
                            if !halfspace.has_inside(&vertex) {
                                continue;
                            }
                            contacts.push((halfspace.normal, *vertex, *bodyid, None));
                        }
                    }
                }
                CollisionGeometry::Sphere(sphere) => {
                    for halfspace in &state.halfspaces {
                        if let Some(contact_point) = sphere.contact_halfspace(halfspace) {
                            contacts.push((halfspace.normal, contact_point, *bodyid, None));
                        }
                    }
                }
                _ => {}
            }
        }
    }

    // Handle contacts between body colliders
    for (i, (bodyid, body)) in izip!(state.treejointids.iter(), state.bodies.iter()).enumerate() {
        for (other_bodyid, other_body) in izip!(
            state.treejointids.iter().skip(i + 1),
            state.bodies.iter().skip(i + 1)
        ) {
            if let (Some(collider), Some(other_collider)) = (&body.collider, &other_body.collider) {
                if !collider.enabled || !other_collider.enabled {
                    continue;
                }
                match &collider.geometry {
                    CollisionGeometry::Cuboid(cuboid) => match &other_collider.geometry {
                        CollisionGeometry::Cuboid(other_cuboid) => {
                            let mut collision_detector =
                                CollisionDetector::new(&cuboid, &other_cuboid);
                            if !collision_detector.gjk() {
                                continue;
                            }
                            let (cp_a, cp_b) = collision_detector.epa();

                            // Contact frame normal
                            let n = UnitVector3::new_normalize(cp_b - cp_a);

                            contacts.push((n, cp_a, *bodyid, Some(*other_bodyid)));
                        }
                        _ => {}
                    },
                    CollisionGeometry::Mesh(mesh) => match &other_collider.geometry {
                        CollisionGeometry::Mesh(other_mesh) => {
                            // TODO: set tolerance according to feature size
                            let mesh_mesh_contacts = mesh_mesh_collision(mesh, other_mesh, 1e-2);

                            for (cp, n) in mesh_mesh_contacts.iter() {
                                contacts.push((*n, *cp, *bodyid, Some(*other_bodyid)));
                            }
                        }
                        _ => {}
                    },
                    CollisionGeometry::Sphere(_) => {}
                };
            }
        }
    }

    contacts
}

fn solve_constraint_and_contact(
    constraint_Js: &Vec<DMatrix<Float>>,
    contact_Js: &Vec<Matrix3xX<Float>>,
    v_free: &DVector<Float>,
    mass_matrix: DMatrix<Float>,
) -> DVector<Float> {
    if constraint_Js.len() == 0 && contact_Js.len() == 0 {
        v_free.clone()
    } else {
        let mut rows: Vec<Matrix1xX<Float>> = vec![];
        for contact_J in contact_Js.iter() {
            rows.extend(contact_J.row_iter().map(|r| r.into_owned()));
        }
        for constraint_J in constraint_Js.iter() {
            rows.extend(constraint_J.row_iter().map(|r| r.into_owned()));
        }
        let J = DMatrix::from_rows(&rows);

        // Formulate the second-order cone programming problem and solve it
        // Ref: Contact Models in Robotics: a Comparative Analysis, 2024,
        // Quentin Le Lidec and et al. III B. Cone Complementarity Problem
        //
        // Also accounting for the joint constraints.
        // Ref: Proximal and Sparse Resolution of Constrained Dynamic
        // Equations, 2021, Justin Carpentier and et al. II. BACKGROUND
        // Lagrangian of the constrained dynamics
        let mass_matrix_lu = mass_matrix.lu();
        let mass_inv = mass_matrix_lu
            .try_inverse()
            .expect("mass matrix not invertible");

        let G: DMatrix<Float> = &J * mass_inv * J.transpose();
        let P = CscMatrix::from(G.row_iter());
        let g = &J * v_free;
        let q: Vec<Float> = Vec::from(g.as_slice());

        // model contact impulse friction restriction
        let n_contacts = contact_Js.len();
        let mut A_triplets: Vec<(usize, usize, Float)> = vec![];
        let mu = 0.95; // TODO: special handling for zero friction
        for i in 0..n_contacts {
            let index = i * 3;
            A_triplets.push((index, index, -mu));
            A_triplets.push((index + 1, index + 1, -1.0));
            A_triplets.push((index + 2, index + 2, -1.0));
        }
        let A = CscMatrix::new_from_triplets(
            n_contacts * 3, // num of constraints
            g.len(),        // num of degrees of lambda
            A_triplets.iter().map(|x| x.0).collect(),
            A_triplets.iter().map(|x| x.1).collect(),
            A_triplets.iter().map(|x| x.2).collect(),
        );

        let b = vec![0.0; n_contacts * 3];
        let cones: Vec<SupportedConeT<Float>> = vec![SecondOrderConeT(3); n_contacts];

        let settings: DefaultSettings<Float> = DefaultSettingsBuilder::default()
            .verbose(false)
            .build()
            .unwrap();
        let mut solver = DefaultSolver::new(&P, &q, &A, &b, &cones, settings);
        solver.solve();
        let lambda = DVector::from(solver.solution.x);

        // Note: with f64 for Clarabel, this might not be needed.
        // Fall back to linear solver.
        // if lambda.iter().any(|x| x.is_nan()) {
        //     if contact_Js.len() == 0 {
        //         let cholesky = G
        //             .clone()
        //             .cholesky()
        //             .expect(&format!("G not positive definite: {}", G));
        //         lambda = -cholesky.solve(&g);
        //     } else {
        //         panic!("lambda contains NaN: {:?}", lambda);
        //     }
        // }

        if lambda.iter().any(|x| x.is_nan()) {
            panic!("lambda contains NaN: {:?}", lambda);
        }

        let impulse = J.transpose() * lambda;
        let v_next: DVector<Float> = v_free
            + mass_matrix_lu
                .solve(&impulse)
                .expect("Failed to solve Mx = J^T λ");
        v_next
    }
}

/// Given contact information, compute the contact Jacobian that transforms
/// generalized velocity to contact frame velocity.
/// Contact frame normal pointing towards body, and away from other body
pub fn compose_contact_jacobian(
    state: &MechanismState,
    normal: &UnitVector3<Float>,
    contact_point: &Vector3<Float>,
    bodyid: usize,
    other_bodyid: Option<usize>, // None if the other body is static, such as halfspaces
    dof: usize,                  // degrees of freedom of system
    blocks: &Vec<Matrix6xX<Float>>, // Jacobian blocks
) -> Matrix3xX<Float> {
    // t and b are contact frame tangential directions
    let t = {
        let candidate = normal.cross(&Vector3::x_axis());
        if candidate.norm() != 0.0 {
            UnitVector3::new_normalize(candidate)
        } else {
            UnitVector3::new_normalize(normal.cross(&Vector3::y_axis()))
        }
    };
    let b = UnitVector3::new_normalize(normal.cross(&t));
    let C = Matrix3::from_rows(&[normal.transpose(), t.transpose(), b.transpose()]);

    // All the bodyids in the linkage from this body to root/world
    let mut linked_bodyids = HashSet::new();
    let mut currentid = bodyid;
    while currentid != 0 {
        linked_bodyids.insert(currentid);
        currentid = state.parents[currentid - 1];
    }

    // All the bodyids in the linkage from other body to root/world
    let mut linked_other_bodyids = HashSet::new();
    if let Some(other_bodyid) = other_bodyid {
        let mut currentid = other_bodyid;
        while currentid != 0 {
            linked_other_bodyids.insert(currentid);
            currentid = state.parents[currentid - 1];
        }
    }

    // Build the Jacobian that transforms generalized v into
    // world-frame spatial twist
    let mut H = Matrix6xX::zeros(dof);
    let mut col_offset = 0;
    for jointid in state.treejointids.iter() {
        // TODO(fixed_joint): take care of fixed joints? since they do not have an entry in blocks.

        let n_cols = blocks[jointid - 1].ncols();
        if linked_bodyids.contains(jointid) {
            H.columns_mut(col_offset, n_cols)
                .copy_from(&blocks[jointid - 1]);
        } else if linked_other_bodyids.contains(jointid) {
            // TODO: Handle self-collision. It probably just
            // means that this block of H would be zero, since
            // this joint would have no effect on relative
            // contact velocity.
            H.columns_mut(col_offset, n_cols)
                .copy_from(&-&blocks[jointid - 1]);
        }
        col_offset += n_cols;
    }

    // Jacobian that transforms world-frame spatial twist into
    // contact point world-frame velocity
    let mut X = Matrix3x6::zeros();
    let r = contact_point;
    X.columns_mut(0, 3).copy_from(&-skew_symmetric(&r));
    X.columns_mut(3, 3).copy_from(&Matrix3::identity());

    C * X * H
}

/// Solve the second-order cone programming problem for resolving contact
pub fn solve_cone_complementarity(P: &CscMatrix<Float>, g: &DVector<Float>) -> DVector<Float> {
    let q: Vec<Float> = Vec::from(g.as_slice());

    assert!(P.m % 3 == 0);
    let n_constraints = P.m / 3;

    let mut A_triplets: Vec<(usize, usize, Float)> = vec![];
    let mu = 0.95; // TODO: special handling for zero friction
    for i in 0..n_constraints {
        let index = i * 3;
        A_triplets.push((index, index, -mu));
        A_triplets.push((index + 1, index + 1, -1.0));
        A_triplets.push((index + 2, index + 2, -1.0));
    }
    let A = CscMatrix::new_from_triplets(
        P.m,
        P.m,
        A_triplets.iter().map(|x| x.0).collect(),
        A_triplets.iter().map(|x| x.1).collect(),
        A_triplets.iter().map(|x| x.2).collect(),
    );

    let b = vec![0.0; n_constraints * 3];
    let cones: Vec<SupportedConeT<Float>> = vec![SecondOrderConeT(3); n_constraints];

    let settings: DefaultSettings<Float> = DefaultSettingsBuilder::default()
        .verbose(false)
        .build()
        .unwrap();
    let mut solver = DefaultSolver::new(&P, &q, &A, &b, &cones, settings);
    solver.solve();

    DVector::from(solver.solution.x)
}

#[cfg(test)]
mod dynamics_tests {

    use crate::{
        assert_close, assert_vec_close,
        contact::HalfSpace,
        integrators::Integrator,
        joint::{
            floating::FloatingJoint,
            prismatic::{JointSpring, PrismaticJoint},
            JointPosition, ToJointTorqueVec,
        },
        simulate::simulate,
        util::assert_close,
        WORLD_FRAME,
    };
    use na::{dvector, vector, Isometry3, Matrix3};

    use crate::{
        double_pendulum::SimpleDoublePendulum,
        helpers::{build_double_pendulum, build_pendulum},
        inertia::SpatialInertia,
        joint::{revolute::RevoluteJoint, Joint, ToJointPositionVec, ToJointVelocityVec},
        rigid_body::RigidBody,
        util::assert_dvec_close,
        GRAVITY, PI,
    };

    use super::*;

    #[test]
    fn dynamics_rod_pendulum_horizontal() {
        // Arrange
        let m = 5.0; // Mass of rod
        let l: Float = 7.0; // Length of rod

        let moment_x = 0.0;
        let moment_y = 1.0 / 3.0 * m * l * l;
        let moment_z = 1.0 / 3.0 * m * l * l;
        let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
        let cross_part = vector![m * l / 2.0, 0.0, 0.0];

        let rod_to_world = Isometry3::identity(); // transformation from rod to world frame
        let axis = Vector3::y_axis(); // axis of joint rotation

        let mut state =
            crate::helpers::build_pendulum(&m, &moment, &cross_part, &rod_to_world, axis);

        // Act
        let joint_accels = dynamics_continuous(&mut state, &vec![0.0].to_joint_torque_vec());

        // Assert
        assert_eq!(
            joint_accels.to_float_dvec(),
            dvector![3.0 * GRAVITY / (2.0 * l)]
        );
    }

    /// world frame ^z       rod/joint frame ^y
    ///             |  / y                   |
    ///             | /                      |
    ///             |/                       |
    ///             +----> x                 +---->x
    #[test]
    fn dynamics_rod_pendulum_horizontal_rotated_frame() {
        // Arrange
        let m = 5.0; // Mass of rod
        let l: Float = 7.0; // Length of rod

        let moment_x = 0.0;
        let moment_y = 1.0 / 3.0 * m * l * l;
        let moment_z = 1.0 / 3.0 * m * l * l;
        let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
        let cross_part = vector![m * l / 2.0, 0.0, 0.0];

        let rod_to_world = Isometry3::rotation(Vector3::x_axis().scale(PI / 2.0));
        let axis = Vector3::z_axis();

        let mut state = build_pendulum(&m, &moment, &cross_part, &rod_to_world, axis);

        // Act
        let joint_accels = dynamics_continuous(&mut state, &vec![0.0].to_joint_torque_vec());

        // Assert
        assert_eq!(
            joint_accels.to_float_dvec(),
            dvector![-3.0 * GRAVITY / (2.0 * l)]
        );
    }

    /// world frame ^z       rod/joint frame ^y
    ///             |  / y                   |
    ///             | /                      |
    ///             |/                       |
    ///             +----> x                 +---->x
    /// separated by some distance in x
    #[test]
    fn dynamics_rod_pendulum_horizontal_moved_frame() {
        // Arrange
        let m = 5.0; // Mass of rod
        let l: Float = 7.0; // Length of rod
        let d: Float = 11.0;

        let moment_x = 0.0;
        let moment_y = 1.0 / 3.0 * m * l * l;
        let moment_z = 1.0 / 3.0 * m * l * l;
        let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
        let cross_part = vector![m * l / 2.0, 0.0, 0.0];

        let rod_to_world = Isometry3::new(vector![d, 0., 0.], Vector3::x_axis().scale(PI / 2.0));
        let axis = Vector3::z_axis();

        let mut state = build_pendulum(&m, &moment, &cross_part, &rod_to_world, axis);

        // Act
        let joint_accels = dynamics_continuous(&mut state, &vec![0.0].to_joint_torque_vec());

        // Assert
        let error = (joint_accels.to_float_dvec() - dvector![-3.0 * GRAVITY / (2.0 * l)]).abs();
        assert!(error < dvector![1e-6], "\nError too large: {:#?}", error);
        // Note: Needs to check for close equality because of numerical error
        // TODO: Handle this such that can check for exact equality?
        // assert_eq!(joint_accels, dvector![-3.0 * GRAVITY / (2.0 * l)]);
    }

    /// Apply a torque that should hold the rod at horizontal position
    #[test]
    fn dynamics_hold_horizontal_rod_pendulum() {
        // Arrange
        let m = 5.0; // Mass of rod
        let l: Float = 7.0; // Length of rod

        let moment_x = 0.0;
        let moment_y = 1.0 / 3.0 * m * l * l;
        let moment_z = 1.0 / 3.0 * m * l * l;
        let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
        let cross_part = vector![m * l / 2.0, 0.0, 0.0];

        let rod_to_world = Isometry3::identity(); // transformation from rod to world frame
        let axis = Vector3::y_axis(); // axis of joint rotation

        let mut state =
            crate::helpers::build_pendulum(&m, &moment, &cross_part, &rod_to_world, axis);

        // Act
        let torque = -m * GRAVITY * l / 2.0;
        let joint_accels = dynamics_continuous(&mut state, &vec![torque].to_joint_torque_vec());

        // Assert
        let error = (joint_accels.to_float_dvec() - dvector![0.0]).abs();
        assert!(
            error < dvector![1e-6],
            "\njoint accel error too large: {:#?}",
            error
        );
    }

    /// A pendulum of point mass
    #[test]
    fn dynamics_simple_pendulum_horizontal() {
        // Arrange
        let m = 5.0;
        let l: Float = 7.0;

        let moment_x = 0.0;
        let moment_y = m * l * l;
        let moment_z = m * l * l;
        let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
        let cross_part = vector![m * l, 0., 0.];

        let rod_to_world = Isometry3::identity();
        let axis = Vector3::y_axis(); // axis of joint rotation

        let mut state =
            crate::helpers::build_pendulum(&m, &moment, &cross_part, &rod_to_world, axis);

        // Act
        let joint_accels = dynamics_continuous(&mut state, &vec![0.0].to_joint_torque_vec());

        // Assert
        assert_eq!(joint_accels.to_float_dvec(), dvector![GRAVITY / l]);
    }

    /// Release a double pendulum (point masses on massless rods) from
    /// horizontal position.
    #[test]
    fn dynamics_double_pendulum_horizontal() {
        // Arrange
        let m = 5.0;
        let l: Float = 7.0;

        let moment_x = 0.0;
        let moment_y = m * l * l;
        let moment_z = m * l * l;
        let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
        let cross_part = vector![m * l, 0., 0.];

        let rod1_to_world = Isometry3::identity();
        let rod2_to_rod1 = Isometry3::translation(l, 0., 0.);
        let axis = Vector3::y_axis(); // axis of joint rotation

        let treejoints = vec![
            Joint::RevoluteJoint(RevoluteJoint::new(
                Transform3D::new("rod1", "world", &rod1_to_world),
                axis.clone(),
            )),
            Joint::RevoluteJoint(RevoluteJoint::new(
                Transform3D::new("rod2", "rod1", &rod2_to_rod1),
                axis.clone(),
            )),
        ];
        let bodies = vec![
            RigidBody::new(SpatialInertia {
                frame: "rod1".to_string(),
                moment: moment.clone(),
                cross_part: cross_part.clone(),
                mass: m,
            }),
            RigidBody::new(SpatialInertia {
                frame: "rod2".to_string(),
                moment: moment.clone(),
                cross_part: cross_part.clone(),
                mass: m,
            }),
        ];
        let mut state = MechanismState::new(treejoints, bodies);

        // Act
        let joint_accels = dynamics_continuous(&mut state, &vec![0.0, 0.0].to_joint_torque_vec());

        // Assert
        assert_dvec_close(
            &joint_accels.to_float_dvec(),
            &dvector![GRAVITY / l, -GRAVITY / l],
            1e-6,
        );
    }

    #[test]
    fn double_pendulum_dynamics() {
        // Arrange
        let m = 3.0;
        let l: Float = 5.0;

        let moment_x = m * l * l;
        let moment_y = m * l * l;
        let moment_z = 0.;
        let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
        let cross_part = vector![0., 0., -m * l];

        let rod1_to_world = Isometry3::identity();
        let rod2_to_rod1 = Isometry3::translation(0., 0., -l);
        let axis = Vector3::y_axis();

        let mut state = build_double_pendulum(
            &m,
            &moment,
            &cross_part,
            &rod1_to_world,
            &rod2_to_rod1,
            &axis,
        );

        let q1 = 3.0;
        let q2 = 5.0;
        let q1dot = 3.0;
        let q2dot = 5.0;
        let q_init = vec![q1, q2].to_joint_pos_vec();
        let v_init = vec![q1dot, q2dot].to_joint_vel_vec();
        state.update(&q_init, &v_init);

        // Act
        let vdot = dynamics_continuous(&mut state, &vec![0.0, 0.0].to_joint_torque_vec());

        // Assert
        let double_pendulum = SimpleDoublePendulum::new(m, m, l, l, q1, q2, q1dot, q2dot);
        let vdot_ref = DVector::from_column_slice(double_pendulum.dynamics().as_slice());
        assert_vec_close!(&vdot.to_float_dvec(), &vdot_ref, 1e-5);
    }

    #[test]
    fn spring_on_frictionless_ground() {
        // Arrange
        let m = 1.0;
        let r = 0.1;
        let l_init = 2.0;
        let l_rest = l_init / 3.0;

        let frame_A = "A";
        let A = RigidBody::new_sphere(m, r, &frame_A);

        let frame_B = "B";
        let B = RigidBody::new_sphere(m, r, &frame_B);

        let A_to_world = Transform3D::identity(&frame_A, WORLD_FRAME);
        let B_to_A = Transform3D::identity(&frame_B, &frame_A);

        let treejoints = vec![
            Joint::FloatingJoint(FloatingJoint::new(A_to_world)),
            Joint::PrismaticJoint(PrismaticJoint::new_with_spring(
                B_to_A,
                vector![1., 0., 0.],
                JointSpring { k: 50.0, l: l_rest },
            )),
        ];

        let mut state = MechanismState::new(treejoints, vec![A, B]);
        let spring_joint: usize = 2;
        state.set_joint_q(spring_joint, JointPosition::Float(l_init));

        let h_ground = 0.0;
        let alpha = 1.0;
        let mu = 0.0; // frictionless
        let ground = HalfSpace::new_with_params(Vector3::z_axis(), h_ground, alpha, mu);
        state.add_halfspace(ground);

        // Act
        let final_time = 1.1;
        let dt = 1e-3;
        let (_q, _v) = simulate(
            &mut state,
            final_time,
            dt,
            |_state| vec![],
            &Integrator::RungeKutta4,
        );

        // Assert
        let poses = state.poses();
        let pose_A = &poses[0];
        let pose_B = &poses[1];

        let x_A = pose_A.translation.x;
        let x_B = pose_B.translation.x;
        let x_midpoint = (x_A + x_B) / 2.0;
        assert_close(x_midpoint, l_init / 2.0, 1e-5);
        assert!(x_A > 0.0);
        assert!(x_B < l_init);
    }

    /// A floating-base turning a point mass at end of rod
    #[test]
    fn motor_turning_mass() {
        // Arrange
        let base_frame = "base";
        let m_base = 1.0;
        let r_base = 1.0;
        let base = RigidBody::new_sphere(m_base, r_base, base_frame);
        let base_to_world = Transform3D::identity(base_frame, WORLD_FRAME);

        let m = 1.0;
        let r = 1.0;
        let moment_x = m * r * r;
        let moment_y = 0.0;
        let moment_z = moment_x;
        let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
        let cross_part = vector![0., -m * r, 0.];
        let mass_frame = "mass";
        let mass = RigidBody::new(SpatialInertia::new(moment, cross_part, m, &mass_frame));
        let mass_to_base = Transform3D::identity(&mass_frame, &base_frame);
        let motor_axis = Vector3::z_axis();

        let bodies = vec![base, mass];
        let treejoints = vec![
            Joint::FloatingJoint(FloatingJoint::new(base_to_world)),
            Joint::RevoluteJoint(RevoluteJoint::new(mass_to_base, motor_axis)),
        ];
        let mut state = MechanismState::new(treejoints, bodies);

        // Act
        let motor_torque = 1.0;
        let tau = vec![
            JointTorque::Spatial(SpatialVector {
                angular: vector![0.0, 0., 0.0],
                linear: vector![0.0, 0., 0.],
            }),
            JointTorque::Float(motor_torque),
        ];
        let acc = dynamics_continuous(&mut state, &tau);

        // Assert
        let base_moment_x = 2.0 / 5.0 * m_base * r_base * r_base;
        let base_angular = -motor_torque / base_moment_x;
        assert_vec_close!(
            acc[0].spatial().angular,
            vector![0., 0., base_angular],
            1e-5
        );
        let base_linear_x = -motor_torque / r / m_base;
        assert_vec_close!(
            acc[0].spatial().linear,
            vector![base_linear_x, 0.0, -GRAVITY],
            1e-5
        );
        assert_close!(
            acc[1].float(),
            -base_angular + motor_torque / moment_x + -base_linear_x * r,
            1e-5
        );
    }
}
