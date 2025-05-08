use std::collections::HashMap;
use std::ops::Mul;

use itertools::izip;
use na::{UnitVector3, Vector3};

use crate::collision::CollisionDetector;
use crate::spatial::transform::Transform3D;
use crate::spatial::twist::Twist;
use crate::spatial::wrench::Wrench;
use crate::WORLD_FRAME;
use crate::{control::energy_control::spring_force, mechanism::MechanismState, types::Float};

#[derive(Clone, PartialEq, Debug)]
pub struct ContactPoint {
    pub frame: String, // the frame the contact point is expressed in
    pub location: Vector3<Float>,
    pub k: Float, // Spring constant
}

impl ContactPoint {
    pub fn new(frame: &str, location: Vector3<Float>) -> Self {
        ContactPoint {
            frame: frame.to_string(),
            location,
            k: 50e3,
        }
    }

    pub fn new_with_k(frame: &str, location: Vector3<Float>, k: Float) -> Self {
        ContactPoint {
            frame: frame.to_string(),
            location,
            k,
        }
    }

    /// Transform the contact point to be expressed in the "to" frame of transform
    pub fn transform(&self, transform: &Transform3D) -> Self {
        if self.frame != transform.from {
            panic!(
                "current frame {} != transform from frame {}",
                self.frame, transform.from
            );
        }

        let rot = transform.rot();
        let trans = transform.trans();
        let location = rot.mul(&self.location) + trans;

        ContactPoint {
            frame: transform.to.clone(),
            location,
            k: self.k,
        }
    }

    pub fn inside_halfspace(&self, halfspace: &HalfSpace) -> bool {
        halfspace.has_inside(&self.location)
    }

    pub fn compute_contact(&self, halfspace: &HalfSpace) -> (Float, UnitVector3<Float>) {
        let penetration = -(self.location - halfspace.point).dot(&halfspace.normal);
        let normal = halfspace.normal;
        (penetration, normal)
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct HalfSpace {
    pub point: Vector3<Float>,      // A point in the half-space
    pub normal: UnitVector3<Float>, // Outward normal direction of the half-space
    pub alpha: Float, // roughly how much velocity is lost, coefficient of restitution e ~= 1-a*v_in
    pub mu: Float,    // coefficient of friction
}

impl HalfSpace {
    // Create a half-space that is moved along normal by distance, from origin
    pub fn new(normal: UnitVector3<Float>, distance: Float) -> Self {
        HalfSpace {
            point: normal.scale(distance),
            normal,
            alpha: 0.9,
            mu: 0.5,
        }
    }

    pub fn new_with_params(
        normal: UnitVector3<Float>,
        distance: Float,
        alpha: Float,
        mu: Float,
    ) -> Self {
        HalfSpace {
            point: normal.scale(distance),
            normal,
            alpha,
            mu,
        }
    }

    // True if the point is inside the half-space
    pub fn has_inside(&self, point: &Vector3<Float>) -> bool {
        (point - self.point).dot(&self.normal) <= 0.0
    }
}

/// An ideal spring contact, that acts against half-spaces, to model a spring
/// attached to the rigid body. Particularly useful to SLIP model.
#[derive(Clone, PartialEq, Debug)]
pub struct SpringContact {
    pub frame: String,                 // frame the spring contact is attached to
    pub l_rest: Float,                 // rest length
    pub direction: UnitVector3<Float>, // direction of the spring, expressed in body frame
    pub k: Float,                      // spring constant
    pub registered_contact: Option<Vector3<Float>>, // a point in contact, in world frame
    pub registered_halfspace: Option<HalfSpace>, // halfspace the spring is in contact with
}

impl SpringContact {
    pub fn new(frame: &str, l_rest: Float, direction: UnitVector3<Float>, k: Float) -> Self {
        SpringContact {
            frame: frame.to_string(),
            l_rest,
            direction,
            k,
            registered_contact: None,
            registered_halfspace: None,
        }
    }
}

/// Compute the contact wrenches due to contacts
pub fn contact_dynamics(
    state: &mut MechanismState,
    bodies_to_root: &HashMap<usize, Transform3D>,
    twists: &HashMap<usize, Twist>,
) -> HashMap<usize, Wrench> {
    let mut contact_wrenches = HashMap::new();
    for (jointid, body) in izip!(state.treejointids.iter(), state.bodies.iter_mut()) {
        let bodyid = jointid;
        let mut wrench = Wrench::zero("world");
        let body_to_root = bodies_to_root.get(bodyid).unwrap();
        let twist = twists.get(bodyid).unwrap();

        // Handle point contacts with halfspaces
        let contact_points = &body.contact_points;
        for contact_point in contact_points {
            let contact_point = contact_point.transform(body_to_root);
            let velocity = twist.point_velocity(&contact_point);
            for halfspace in state.halfspaces.iter() {
                if !contact_point.inside_halfspace(&halfspace) {
                    continue;
                }
                let (penetration, normal) = contact_point.compute_contact(&halfspace);
                let contact_force = calculate_contact_force_halfspace(
                    halfspace,
                    penetration,
                    &velocity,
                    &normal,
                    contact_point.k,
                );
                wrench += Wrench::from_force(&contact_point.location, &contact_force, "world");
            }
        }

        // TODO: make it work with runge-kutta methods. i.e. Make it not requiring
        // modifying state
        // Handle spring contacts with halfspaces
        let spring_contacts = &mut body.spring_contacts;
        for spring_contact in spring_contacts {
            let body_location = body_to_root.trans();
            if spring_contact.registered_contact.is_none() {
                let spring_direction = body_to_root.rot() * *spring_contact.direction;
                let contact_location = body_location + spring_direction * spring_contact.l_rest;
                for halfspace in state.halfspaces.iter() {
                    if !halfspace.has_inside(&contact_location) {
                        continue;
                    }

                    // Take the current spring contact location as the contact
                    // point. It might not be exactly on the halfspace surface.
                    spring_contact.registered_contact = Some(contact_location);
                    spring_contact.registered_halfspace = Some(*halfspace);

                    // A spring contact can only be registered to one halfspace
                    // at a time.
                    break;
                }
            } else {
                // Exert spring force from the registered contact point
                let contact_location = spring_contact.registered_contact.unwrap();

                let spring_direction =
                    UnitVector3::new_normalize(contact_location - body_location).into_inner();

                let halfspace = spring_contact.registered_halfspace.unwrap();
                if spring_direction.dot(&halfspace.normal) > 0.0 {
                    panic!("Spring force is into the halfspace!");
                }

                let direction_distance = (contact_location - body_location).norm();

                if direction_distance < spring_contact.l_rest {
                    let spring_force =
                        spring_force(spring_contact.l_rest, direction_distance, spring_contact.k);
                    let force = -spring_direction * spring_force;

                    wrench += Wrench::from_force(&body_location, &force, WORLD_FRAME);
                } else {
                    // Spring no longer under load, detach
                    spring_contact.registered_contact = None;
                    spring_contact.registered_halfspace = None;

                    // spring direction set to the angle at leaving the surface
                    spring_contact.direction = UnitVector3::new_normalize(spring_direction);

                    // Note: Spring length set to zero to avoid tripping
                    // But now it seems not necessary
                    // spring_contact.l_rest = 0.0;
                }
            }
        }

        contact_wrenches.insert(*bodyid, wrench);
    }

    // Compute contacts between body colliders
    for (i, (jointid, body)) in izip!(state.treejointids.iter(), state.bodies.iter()).enumerate() {
        for (next_jointid, next_body) in izip!(
            state.treejointids.iter().skip(i + 1),
            state.bodies.iter().skip(i + 1)
        ) {
            if let (Some(collider), Some(next_collider)) = (&body.collider, &next_body.collider) {
                let mut collision_detector = CollisionDetector::new(&collider, &next_collider);
                if collision_detector.gjk() {
                    let (cp_a, cp_b) = collision_detector.epa();

                    let body_twist = twists.get(jointid).unwrap();
                    let cp_a_vel = body_twist.point_velocity(&ContactPoint::new(WORLD_FRAME, cp_a));
                    let next_body_twist = twists.get(next_jointid).unwrap();
                    let cp_b_vel =
                        next_body_twist.point_velocity(&ContactPoint::new(WORLD_FRAME, cp_b));
                    let velocity = cp_a_vel - cp_b_vel;

                    let f_b_to_a = calculate_contact_force_collider(&cp_a, &cp_b, &velocity);
                    let wrench_b_to_a = Wrench::from_force(&cp_a, &f_b_to_a, WORLD_FRAME);
                    let wrench_a_to_b = Wrench::from_force(&cp_b, &-f_b_to_a, WORLD_FRAME);

                    if let Some(wrench) = contact_wrenches.get_mut(jointid) {
                        *wrench += wrench_b_to_a
                    } else {
                        contact_wrenches.insert(*jointid, wrench_b_to_a);
                    }
                    if let Some(wrench) = contact_wrenches.get_mut(next_jointid) {
                        *wrench += wrench_a_to_b
                    } else {
                        contact_wrenches.insert(*next_jointid, wrench_a_to_b);
                    }
                }
            }
        }
    }

    contact_wrenches
}

/// Computes the contact force between two shapes A & B
/// Ref: Normal force
///     1. Coeﬀicient of restitution interpreted as damping in vibroimpact,
///         K. H. Hunt and F. R. E. Crossley, 1975
///     2. A Compliant Contact Model with Nonlinear Damping for Simulation of Robotic Systems,
///         D. W. Marhefka and D. E. Orin, 1999
/// Ref: Tangential friction
///     1. A Transition-Aware Method for the Simulation of Compliant Contact with Regularized Friction,
///         Castro, Alejandro M., et al., 2020
///     2. Drake: Modeling of Dry Friction
///         https://drake.mit.edu/doxygen_cxx/group__friction__model.html
///
/// Returns the force from B towards A
fn calculate_contact_force(
    penetration: Float,        // Depth of AB intersection, value > 0
    normal: &Vector3<Float>,   // Direction pointing towards A
    velocity: &Vector3<Float>, // Velocity of contact point on A relative to point on B
    k_A: Float,                // Spring constant of A
    k_B: Float,                // Spring constant of B
    alpha: Float, // how much velocity is lost, => coefficient of restitution e ~= 1-alpha*v_in
    mu: Float,    // Coefficient of friction
) -> Vector3<Float> {
    let z = penetration;
    let z_dot = -velocity.dot(normal);

    // Hunt-Crossley model for normal force
    let zn = z.powf(3.0 / 2.0);
    let k = k_A * k_B / (k_A + k_B);
    let a = alpha;
    let λ = 3.0 / 2.0 * a * k;
    let π = (λ * zn * z_dot + k * zn).max(0.0);
    let f_normal = π * *normal;

    // friction force in tangential direction - regularized coulomb friction
    let v_t = velocity + z_dot * *normal; // tangential velocity
    let v_t_norm = v_t.norm();
    let f_friction = {
        if v_t_norm == 0.0 {
            Vector3::zeros()
        } else {
            let v_s = 1e-3; // slip tolerance, amount of velocity allowed for contact that should be stationary
            let s = v_t_norm / v_s;
            let μ = mu;
            let μ = {
                if s > 1.0 {
                    μ
                } else {
                    μ * s
                }
            };
            -μ * π * (v_t / v_t_norm)
        }
    };

    f_normal + f_friction
}

/// Computes contact force between two colliders
/// Returns force from B to A
pub fn calculate_contact_force_collider(
    cp_a: &Vector3<Float>,     // Contact point on body A
    cp_b: &Vector3<Float>,     // Contact point on body B
    velocity: &Vector3<Float>, // Velocity of point A relative to point B
) -> Vector3<Float> {
    let penetration = cp_b - cp_a;
    let normal = UnitVector3::new_normalize(penetration);
    let k = 5e3;
    let alpha = 1.0;
    let mu = 1.0;
    return calculate_contact_force(penetration.norm(), &normal, velocity, k, k, alpha, mu);
}

/// Computes contact force between a contact point and a halfspace
/// Returns force from halfspace to the body
pub fn calculate_contact_force_halfspace(
    halfspace: &HalfSpace,
    penetration: Float,
    velocity: &Vector3<Float>,
    normal: &Vector3<Float>,
    k_point: Float, // Spring constant of the point in contact
) -> Vector3<Float> {
    let k_halfspace = 50e3;
    return calculate_contact_force(
        penetration,
        normal,
        velocity,
        k_point,
        k_halfspace,
        halfspace.alpha,
        halfspace.mu,
    );
}

#[cfg(test)]
mod contact_tests {

    use crate::control::Controller;
    use crate::helpers::add_cube_contacts;
    use crate::integrators::Integrator;
    use crate::joint::revolute::RevoluteJoint;
    use crate::spatial::spatial_vector::SpatialVector;
    use crate::spatial::transform::Matrix4Ext;
    use crate::{
        assert_close,
        helpers::{build_SLIP, build_cube, build_rimless_wheel},
        inertia::SpatialInertia,
        interface::controller::NullController,
        joint::{
            floating::FloatingJoint,
            prismatic::{JointSpring, PrismaticJoint},
            Joint, JointPosition, JointVelocity, ToJointTorqueVec,
        },
        rigid_body::RigidBody,
        simulate::step,
        spatial::pose::Pose,
        util::assert_close,
        GRAVITY,
    };
    use na::{dvector, vector, zero, Matrix3, Matrix4, UnitQuaternion};

    use crate::{helpers::build_pendulum, simulate::simulate, util::assert_dvec_close, PI};

    use super::*;

    #[test]
    fn pendulum_hit_ground() {
        // Arrange
        let m = 1.5;
        let l = 10.0;

        let moment_x = 0.0;
        let moment_y = m * l * l;
        let moment_z = m * l * l;
        let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
        let cross_part = vector![m * l, 0., 0.];

        let rod_to_world = Matrix4::identity();
        let axis = vector![0., 1., 0.];

        let mut state = build_pendulum(&m, &moment, &cross_part, &rod_to_world, &axis);
        state.add_contact_point(&ContactPoint::new("rod", vector![l, 0., 0.]));

        state.add_halfspace(&&HalfSpace::new(Vector3::z_axis(), -5.0));

        // Act
        let final_time = 5.0;
        let dt = 1e-2;
        let (qs, _vs) = simulate(
            &mut state,
            final_time,
            dt,
            |_state| vec![0.0].to_joint_torque_vec(),
            &crate::integrators::Integrator::RungeKutta4,
        );

        // Assert
        let q_final = qs[qs.len() - 1][0].float();
        let q_expect = 30.0 * PI / 180.0; // 30 degrees
        assert_dvec_close(&dvector![*q_final], &dvector![q_expect], 1e-3);
    }

    #[test]
    fn cube_fall_ground() {
        // Arrange
        let m = 3.0;
        let l = 1.0;
        let mut state = build_cube(m, l);

        let h_ground = -10.0;
        state.add_halfspace(&HalfSpace::new(Vector3::z_axis(), h_ground));

        let q_init = vec![JointPosition::Pose(Pose {
            rotation: UnitQuaternion::identity(),
            translation: vector![0.0, 0.0, 0.0],
        })];
        let v_init = vec![JointVelocity::Spatial(SpatialVector {
            angular: vector![1.0, 1.0, 1.0],
            linear: vector![1.0, 1.0, 1.0],
        })];
        state.update(&q_init, &v_init);

        // Act
        let final_time = 5.0;
        let dt = 1e-3;
        let (qs, _vs) = simulate(
            &mut state,
            final_time,
            dt,
            |_state| vec![],
            &Integrator::RungeKutta4,
        );

        // Assert
        let q_final = qs[qs.len() - 1][0].pose();
        assert_close(q_final.translation.z, h_ground + l / 2.0, 1e-2);
    }

    #[test]
    fn cube_slide_ground() {
        // Arrange
        let m = 3.0;
        let l = 1.0;
        let v_x_init = 1.0;
        let mut state = build_cube(m, l);

        let h_ground = -l / 2.0;
        let mu = 0.5;
        let ground = HalfSpace::new_with_params(Vector3::z_axis(), h_ground, 1.0, mu);
        state.add_halfspace(&ground);

        let q_init = vec![JointPosition::Pose(Pose {
            rotation: UnitQuaternion::identity(),
            translation: vector![0.0, 0.0, 0.0],
        })];
        let v_init = vec![JointVelocity::Spatial(SpatialVector {
            angular: vector![0.0, 0.0, 0.0],
            linear: vector![v_x_init, 0.0, 0.0],
        })];
        state.update(&q_init, &v_init);

        // Act
        let final_time = 2.0;
        let dt = 1e-3;
        let (qs, vs) = simulate(
            &mut state,
            final_time,
            dt,
            |_state| vec![],
            &Integrator::RungeKutta4,
        );

        // Assert
        let q_final = qs[qs.len() - 1][0].pose();
        assert_close!(q_final.translation.z, 0.0, 1e-2);

        let v_final = vs[vs.len() - 1][0].spatial();
        assert_close!(v_final.linear.norm(), 0.0, 5e-3);
        assert_close!(v_final.angular.norm(), 0.0, 1e-2);

        let acc_friction = -GRAVITY * mu;
        let sliding_t = v_x_init / -acc_friction;
        let sliding_x = v_x_init * sliding_t + acc_friction * sliding_t.powi(2) / 2.0;
        assert_close!(q_final.translation.x, sliding_x, 1e-2);
    }

    /// Hit the ground with rotational and translational velocity
    #[test]
    fn cube_hit_ground() {
        // Arrange
        let m = 3.0;
        let l = 1.0;
        let mut state = build_cube(m, l);

        let h_ground = -10.0;
        state.add_halfspace(&HalfSpace::new(Vector3::z_axis(), h_ground));

        let q_init = vec![JointPosition::Pose(Pose {
            rotation: UnitQuaternion::identity(),
            translation: vector![0.0, 0.0, 0.0],
        })];
        let v_init = vec![JointVelocity::Spatial(SpatialVector {
            angular: vector![0.0, 5.0, 0.0],
            linear: vector![1.0, 0.0, 0.0],
        })];
        state.update(&q_init, &v_init);

        // Act
        let final_time = 5.0;
        let dt = 1e-3;
        let (qs, vs) = simulate(
            &mut state,
            final_time,
            dt,
            |_state| vec![],
            &Integrator::RungeKutta4,
        );

        // Assert
        let q_final = qs[qs.len() - 1][0].pose();
        assert_close(q_final.translation.z, h_ground + l / 2.0, 1e-2);

        let v_final = vs[vs.len() - 1][0].spatial();
        assert_close(v_final.linear.norm(), 0.0, 5e-3);
        assert_close(v_final.angular.norm(), 0.0, 1e-2);
    }

    #[test]
    fn two_cubes_hit_ground() {
        // Arrange
        let m = 3.0;
        let l = 1.0;

        let cube1_frame = "cube 1";
        let cube1 = RigidBody::new_cube(m, l, cube1_frame);

        let cube2_frame = "cube 2";
        let cube2 = RigidBody::new_cube(m, l, cube2_frame);
        let bodies = vec![cube1, cube2];

        let cube1_to_world = Transform3D::identity(&cube1_frame, WORLD_FRAME);
        let cube2_to_world = Transform3D::identity(&cube2_frame, WORLD_FRAME);
        let treejoints = vec![
            Joint::FloatingJoint(FloatingJoint::new(cube1_to_world)),
            Joint::FloatingJoint(FloatingJoint::new(cube2_to_world)),
        ];
        let mut state = MechanismState::new(treejoints, bodies);
        add_cube_contacts(&mut state, &cube1_frame, l);
        add_cube_contacts(&mut state, &cube2_frame, l);

        let h_ground = -10.0;
        state.add_halfspace(&HalfSpace::new(Vector3::z_axis(), h_ground));

        let q_init = vec![
            JointPosition::Pose(Pose {
                rotation: UnitQuaternion::identity(),
                translation: vector![2.0 * l, 0.0, 0.0],
            }),
            JointPosition::Pose(Pose {
                rotation: UnitQuaternion::identity(),
                translation: vector![-2.0 * l, 0.0, 0.0],
            }),
        ];
        let v_init = vec![
            JointVelocity::Spatial(SpatialVector {
                angular: vector![0.0, 5.0, 0.0],
                linear: vector![1.0, 0.0, 0.0],
            }),
            JointVelocity::Spatial(SpatialVector {
                angular: vector![0.0, -5.0, 0.0],
                linear: vector![-1.0, 0.0, 0.0],
            }),
        ];
        state.update(&q_init, &v_init);

        // Act
        let final_time = 5.0;
        let dt = 1e-3;
        let (qs, vs) = simulate(
            &mut state,
            final_time,
            dt,
            |_state| vec![],
            &Integrator::RungeKutta4,
        );

        // Assert
        let q1_final = qs[qs.len() - 1][0].pose();
        let q2_final = qs[qs.len() - 1][1].pose();
        assert_close!(q1_final.translation.z, h_ground + l / 2.0, 1e-2);
        assert_close!(q2_final.translation.z, h_ground + l / 2.0, 1e-2);

        let v1_final = vs[vs.len() - 1][0].spatial();
        let v2_final = vs[vs.len() - 1][1].spatial();

        assert_close!(v1_final.linear.norm(), 0.0, 5e-3);
        assert_close!(v1_final.angular.norm(), 0.0, 1e-2);
        assert_close!(v2_final.linear.norm(), 0.0, 5e-3);
        assert_close!(v2_final.angular.norm(), 0.0, 1e-2);
    }

    #[test]
    fn mass_hit_ground() {
        // Arrange
        let m = 1.0;
        let r = 0.1;
        let v_x_init = 2.0;

        let ball_frame = "ball";
        let world_frame = "world";
        let ball_to_world = Transform3D::identity(&ball_frame, &world_frame);

        let ball = RigidBody::new_sphere(m, r, &ball_frame);

        let treejoints = vec![Joint::FloatingJoint(FloatingJoint::new(ball_to_world))];
        let bodies = vec![ball];
        let mut state = MechanismState::new(treejoints, bodies);

        let h_ground = -0.3;
        let alpha = 1.0;
        let mu = 0.5;
        let ground = HalfSpace::new_with_params(Vector3::z_axis(), h_ground, alpha, mu);
        state.add_halfspace(&ground);

        state.add_contact_point(&ContactPoint::new(ball_frame, vector![0.0, 0.0, 0.0]));

        state.set_joint_v(
            1,
            JointVelocity::Spatial(SpatialVector {
                angular: vector![0.0, 0.0, 0.0],
                linear: vector![v_x_init, 0.0, 0.0],
            }),
        );

        // Act
        let final_time = 2.0;
        let dt = 1e-3;
        let (qs, vs) = simulate(
            &mut state,
            final_time,
            dt,
            |_state| vec![],
            &Integrator::RungeKutta4,
        );

        // Assert
        let q_final = qs[qs.len() - 1][0].pose();

        // Check mass on the ground
        assert_close!(q_final.translation.z, h_ground, 1e-2);

        // Check mass stationary
        let v_final = vs[vs.len() - 1][0].spatial();
        assert_close!(v_final.linear.norm(), 0.0, 1e-2);
        assert_close!(v_final.angular.norm(), 0.0, 1e-3);

        // Check mass x = where it hits the ground + sliding distance
        let ground_hit_t = (-h_ground * 2.0 / GRAVITY).sqrt(); // h = 1/2*a*t^2
        let ground_hit_x = ground_hit_t * v_x_init;

        let v_z_hit = GRAVITY * ground_hit_t;
        let v_x_reduction = v_z_hit * mu; // m*v = integral F dt; friciion = normal force * mu
        let v_x_after = v_x_init - v_x_reduction; // velocity after hitting ground
        assert!(v_x_after > 0.0); // otherwise the above calculation doesn't work, since friction must have changed direction

        let acc_friction = -GRAVITY * mu;
        let sliding_t = v_x_after / -acc_friction;
        let sliding_x = v_x_after * sliding_t + acc_friction * sliding_t.powi(2) / 2.0; // ut + 1/2*a*t^2

        let x_expect = ground_hit_x + sliding_x;
        assert_close!(q_final.translation.x, x_expect, 1e-2);
    }

    /// Rimless wheel rolling down a slope, settling into a stable limit cycle
    #[test]
    fn rimless_wheel() {
        // Arrange
        let m_body = 10.0;
        let r_body = 5.0;

        let l = 10.0;
        let n_foot = 8;

        let mut state = build_rimless_wheel(m_body, r_body, l, n_foot);

        let angle: Float = Float::to_radians(10.0);
        let normal = UnitVector3::new_normalize(vector![angle.sin(), 0.0, angle.cos()]);
        let alpha = 0.9;
        let mu = 0.5;
        let h_ground = -20.0;
        state.add_halfspace(&HalfSpace::new_with_params(normal, h_ground, alpha, mu));

        let q_init = vec![JointPosition::Pose(Pose {
            rotation: UnitQuaternion::identity(),
            translation: vector![0.0, 0.0, 0.0],
        })];
        let v_init = vec![JointVelocity::Spatial(SpatialVector {
            angular: vector![0.0, 0.0, 0.0],
            linear: vector![1.0, 0.0, 0.0], // Give the wheel a slight push
        })];
        state.update(&q_init, &v_init);

        // Act
        let final_time = 20.0;
        let dt = 1.0 / 600.0;
        let (qs, vs) = simulate(
            &mut state,
            final_time,
            dt,
            |_state| vec![],
            &Integrator::RungeKutta4,
        );

        // Assert
        let omega_final = vs[vs.len() - 1][0]
            .spatial()
            .angular
            .dot(&Vector3::y_axis());
        assert!(omega_final > 0.0);

        let omega_max = vs
            .iter()
            .map(|v| v[0].spatial().angular.dot(&Vector3::y_axis()))
            .fold(0.0, Float::max);
        assert!(omega_max < 0.75);
    }

    /// Compass gait standing still on a hill
    /// Ref: Underactuated Robotics: 4.2.2 The Compass Gait
    ///     https://underactuated.csail.mit.edu/simple_legs.html#section2
    #[test]
    fn compass_gait_standing() {
        // Arrange
        let m_hip = 10.0;
        let r_hip = 0.3;
        let m_leg = 5.0;
        let l_leg = 1.0;

        let hip_frame = "hip";
        let moment_x = m_hip * r_hip * r_hip * 2.0 / 5.0;
        let hip = RigidBody::new(SpatialInertia {
            frame: hip_frame.to_string(),
            moment: Matrix3::from_diagonal(&vector![moment_x, moment_x, moment_x]),
            cross_part: zero(),
            mass: m_hip,
        });

        let moment_x = m_leg * (l_leg / 2.0) * (l_leg / 2.0);
        let moment_y = moment_x;
        let moment_z = 0.0;
        let cross_part = vector![0., 0., -m_leg * l_leg / 2.0];

        let left_leg_frame = "left leg";
        let left_leg = RigidBody::new(SpatialInertia {
            frame: left_leg_frame.to_string(),
            moment: Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]),
            cross_part,
            mass: m_leg,
        });

        let right_leg_frame = "right leg";
        let right_leg = RigidBody::new(SpatialInertia {
            frame: right_leg_frame.to_string(),
            moment: Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]),
            cross_part,
            mass: m_leg,
        });

        let hip_to_world = Transform3D::identity(hip_frame, WORLD_FRAME);
        let left_leg_to_hip = Transform3D::identity(left_leg_frame, hip_frame);
        let left_leg_axis = vector![0., -1., 0.];
        let right_leg_to_hip = Transform3D::identity(right_leg_frame, hip_frame);
        let right_leg_axis = vector![0., -1., 0.];
        let treejoints = vec![
            Joint::FloatingJoint(FloatingJoint::new(hip_to_world)),
            Joint::RevoluteJoint(RevoluteJoint::new(left_leg_to_hip, left_leg_axis)),
            Joint::RevoluteJoint(RevoluteJoint::new(right_leg_to_hip, right_leg_axis)),
        ];

        let bodies = vec![hip, left_leg, right_leg];

        let mut state = MechanismState::new(treejoints, bodies);
        state.add_contact_point(&ContactPoint::new(left_leg_frame, vector![0., 0., -l_leg]));
        state.add_contact_point(&ContactPoint::new(right_leg_frame, vector![0., 0., -l_leg]));

        let h_ground = 0.0;
        let angle: Float = Float::to_radians(5.0);
        let normal = UnitVector3::new_normalize(vector![angle.sin(), 0.0, angle.cos()]);
        let alpha = 1.0;
        let mu = 1.0;
        let ground = HalfSpace::new_with_params(normal, h_ground, alpha, mu);
        state.add_halfspace(&ground);

        state.set_joint_q(
            1,
            JointPosition::Pose(Pose {
                rotation: UnitQuaternion::identity(),
                translation: vector![0., 0., l_leg],
            }),
        );

        state.set_joint_q(2, JointPosition::Float(Float::to_radians(30.0)));
        state.set_joint_q(3, JointPosition::Float(Float::to_radians(-30.0)));

        // Act
        let final_time = 2.0;
        let dt = 1e-3;
        let (q, v) = simulate(
            &mut state,
            final_time,
            dt,
            |_state| vec![],
            &Integrator::RungeKutta4,
        );

        // Assert
        let q_final = q[q.len() - 1][0].pose();
        assert!(q_final.translation.x > 0.0);
        assert!(q_final.translation.z > 0.0);

        let v_final = v[v.len() - 1][0].spatial();
        assert_eq!(v_final.angular.x, 0.0);
        assert_eq!(v_final.angular.z, 0.0);
        assert_close!(v_final.angular.y, 0.0, 1e-3);
        assert_eq!(v_final.linear.y, 0.0);
        assert_close!(v_final.linear.x, 0.0, 5e-3);
        assert_close!(v_final.linear.z, 0.0, 2e-2);
    }

    /// Spring Loaded Inverted Pendulum (SLIP)
    #[test]
    fn SLIP_hopping() {
        // Arrange
        let m = 0.54;
        let r = 0.1;
        let l_rest = 0.2;
        let angle = Float::to_radians(45.0);
        let k_spring = 500.0;

        let direction = UnitVector3::new_normalize(vector![angle.sin(), 0., -angle.cos()]);
        let mut state = build_SLIP(m, r, l_rest, angle, k_spring);

        let h_ground = -0.3;
        state.add_halfspace(&HalfSpace::new(Vector3::z_axis(), h_ground));

        let q_init = vec![JointPosition::Pose(Pose {
            rotation: UnitQuaternion::identity(),
            translation: vector![0.0, 0.0, 0.0],
        })];
        let v_init = vec![JointVelocity::Spatial(SpatialVector {
            angular: vector![0.0, 0.0, 0.0],
            linear: vector![5.0, 0.0, 0.0],
        })];
        state.update(&q_init, &v_init);

        let initial_energy = state.kinetic_energy() + state.gravitational_energy();

        // Act
        let final_time = 3.0;
        let dt = 1.0 / 2000.0;
        let num_steps = (final_time / dt) as usize;
        let mut controller = NullController {};

        let mut v_z_prev = 0.0;
        let mut energies = vec![];
        let mut hs = vec![];
        for _ in 0..num_steps {
            let torque = controller.control(&mut state, None);
            let (q, v) = step(
                &mut state,
                dt,
                &torque,
                &crate::integrators::Integrator::SemiImplicitEuler,
            );

            let v_z = v[0].spatial().linear.z;
            // Apex
            if v_z_prev > 0.0 && v_z <= 0.0 {
                energies.push(state.kinetic_energy() + state.gravitational_energy());
                hs.push(q[0].pose().translation.z);

                // Swing the spring leg to the original front angle
                state.bodies[0].spring_contacts[0].direction = direction;
                state.bodies[0].spring_contacts[0].l_rest = l_rest;
            }
            v_z_prev = v_z
        }

        // Assert
        // Energy conserved
        assert!(energies.len() > 0, "No apex found");
        for energy in energies {
            assert_close!(energy, initial_energy, 1e-2);
        }

        // Hopping height settled
        assert_close!(hs[hs.len() - 3], hs[hs.len() - 2], 1e-3);
        assert_close!(hs[hs.len() - 2], hs[hs.len() - 1], 1e-3);
    }

    // Drop a 2-mass spring onto ground, the spring should settle on the ground
    // with some oscillation
    #[test]
    fn spring_drop() {
        // Arrange
        let m_body = 1.0;
        let r_body = 1.0;
        let m_foot = 1.0;
        let r_foot = 1.0;
        let l_leg = 1.0;
        let l_spring_rest = 0.0;
        let k_spring = 100.0;

        let body_frame = "body";
        let body = RigidBody::new_sphere(m_body, r_body, &body_frame);
        let body_to_world = Transform3D::identity(body_frame, WORLD_FRAME);

        let foot_frame = "foot";
        let foot = RigidBody::new_sphere(m_foot, r_foot, &foot_frame);
        let foot_to_body = Transform3D {
            from: foot_frame.to_string(),
            to: body_frame.to_string(),
            mat: Matrix4::<Float>::move_z(-l_leg),
        };
        let axis_leg = vector![0.0, 0.0, -1.0];

        let spring = JointSpring {
            k: k_spring,
            l: l_spring_rest,
        };
        let treejoints = vec![
            Joint::FloatingJoint(FloatingJoint::new(body_to_world)),
            Joint::PrismaticJoint(PrismaticJoint::new_with_spring(
                foot_to_body,
                axis_leg,
                spring,
            )),
        ];
        let bodies = vec![body, foot];
        let mut state = MechanismState::new(treejoints, bodies);
        state.add_contact_point(&ContactPoint::new(foot_frame, Vector3::zeros()));

        let h_ground = -2.0;
        let alpha = 1.0;
        let mu = 0.0;
        let ground = HalfSpace::new_with_params(Vector3::z_axis(), h_ground, alpha, mu);
        state.add_halfspace(&ground);

        // Act
        let final_time = 4.0;
        let dt = 1e-3;
        let (q, v) = simulate(
            &mut state,
            final_time,
            dt,
            |_state| vec![],
            &Integrator::RungeKutta4,
        );

        // Assert
        let poses = state.poses();

        // Foot on the ground
        let foot_pose = &poses[1];
        assert_eq!(foot_pose.translation.x, 0.0);
        assert_eq!(foot_pose.translation.y, 0.0);
        let z_tol = ((m_body + m_foot) * GRAVITY / 50e3).powf(2.0 / 3.0); // reverse compute the expected penetration into ground
        assert_close!(foot_pose.translation.z, h_ground, z_tol * 2.0);

        // Body upright
        let body_pose = &poses[0];
        assert_eq!(body_pose.rotation, UnitQuaternion::identity());

        // Energy
        let total_energy =
            state.kinetic_energy() + state.gravitational_energy() + state.spring_energy();
        assert_close!(
            total_energy,
            m_foot * GRAVITY * h_ground + m_body * GRAVITY * (h_ground + l_leg),
            1.5
        );
    }

    /// Forward-moving spring dropping onto ground. Body should be leaning forward
    /// at max spring compression.
    #[test]
    fn spring_forward_drop() {
        // Arrange
        let m_body = 1.0;
        let r_body = 1.0;
        let m_foot = 1.0;
        let r_foot = 1.0;
        let l_leg = 1.0;
        let l_spring_rest = 0.0;
        let k_spring = 100.0;

        let body_frame = "body";
        let body = RigidBody::new_sphere(m_body, r_body, &body_frame);
        let body_to_world = Transform3D::identity(body_frame, WORLD_FRAME);

        let foot_frame = "foot";
        let foot = RigidBody::new_sphere(m_foot, r_foot, &foot_frame);
        let foot_to_body = Transform3D {
            from: foot_frame.to_string(),
            to: body_frame.to_string(),
            mat: Matrix4::<Float>::move_z(-l_leg),
        };
        let axis_leg = vector![0.0, 0.0, -1.0];

        let spring = JointSpring {
            k: k_spring,
            l: l_spring_rest,
        };
        let treejoints = vec![
            Joint::FloatingJoint(FloatingJoint::new(body_to_world)),
            Joint::PrismaticJoint(PrismaticJoint::new_with_spring(
                foot_to_body,
                axis_leg,
                spring,
            )),
        ];
        let bodies = vec![body, foot];
        let mut state = MechanismState::new(treejoints, bodies);
        state.add_contact_point(&ContactPoint::new(foot_frame, Vector3::zeros()));

        let h_ground = -2.0;
        let alpha = 1.0;
        let mu = 0.5;
        let ground = HalfSpace::new_with_params(Vector3::z_axis(), h_ground, alpha, mu);
        state.add_halfspace(&ground);

        state.set_joint_v(
            1,
            JointVelocity::Spatial(SpatialVector {
                angular: zero(),
                linear: vector![0.5, 0.0, 0.0],
            }),
        );

        // Act and Assert
        let final_time = 1.5;
        let dt = 1e-3;
        let num_steps = (final_time / dt) as usize;
        let mut prev_v_spring = 0.0;
        let mut had_bottom = false;
        for _ in 0..num_steps {
            let torque = vec![];
            let (q, v) = step(&mut state, dt, &torque, &Integrator::RungeKutta4);

            let v_spring = v[1].float();
            if prev_v_spring < 0.0 && *v_spring >= 0.0 {
                // Bottom time: spring going from compressing to decompressing.
                had_bottom = true;

                let pose_body = q[0].pose();
                let pose_foot = &state.poses()[1];

                // Check that body's orientation is positive about y-axis
                assert_eq!(pose_body.rotation.axis().unwrap(), Vector3::y_axis());
                assert!(pose_body.rotation.angle() > 0.0);

                // Check that body is in front of foot
                assert!(pose_body.translation.x > pose_foot.translation.x);

                break; // only check first bottom time
            }
            prev_v_spring = *v_spring;
        }
        assert_eq!(had_bottom, true); // At least had one bottom time
    }

    /// Put a rod vertically on the ground, give it a small angular deviation, and
    /// its top should fall to the ground, and bottom remain where it was.
    #[test]
    #[ignore] // TODO: make this work
    fn vertical_rod_against_ground() {
        // Arrange
        let m = 1.0;
        let l = 0.5;
        let v_x_init = 1.0;

        let moment_x = m * l * l / 3.0;
        let moment_y = m * l * l / 3.0;
        let moment_z = moment_x / 10.0;
        let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
        let cross_part = vector![0., 0., m * l / 2.0];

        let rod_frame = "rod";
        let rod = RigidBody::new(SpatialInertia {
            frame: rod_frame.to_string(),
            moment,
            cross_part,
            mass: m,
        });

        let rod_to_world = Transform3D::identity(&rod_frame, WORLD_FRAME);
        let treejoints = vec![Joint::FloatingJoint(FloatingJoint::new(rod_to_world))];
        let bodies = vec![rod];
        let mut state = MechanismState::new(treejoints, bodies);

        state.add_contact_point(&ContactPoint::new(rod_frame, vector![0.0, 0.0, 0.0]));

        state.add_contact_point(&ContactPoint::new(rod_frame, vector![0.0, 0.0, l]));

        let alpha = 1.0;
        let mu = 10.0;
        let normal = Vector3::z_axis();
        let h_ground = 0.0;
        let ground = HalfSpace::new_with_params(normal, h_ground, alpha, mu);
        state.add_halfspace(&ground);

        state.set_joint_q(
            1,
            JointPosition::Pose(Pose {
                rotation: UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.001),
                translation: vector![0.0, 0.0, 0.0],
            }),
        );
        state.set_joint_v(
            1,
            JointVelocity::Spatial(SpatialVector {
                angular: Vector3::zeros(),
                linear: vector![v_x_init, 0.0, 0.0],
            }),
        );

        // Act
        let final_time = 2.0;
        let dt = 1e-4;
        let (q, v) = simulate(
            &mut state,
            final_time,
            dt,
            |_state| vec![],
            &Integrator::RungeKutta4,
        );

        // Assert
        let q_final = &q[q.len() - 1][0];
        let pose_body = q_final.pose();
        // rod fall to the ground
        assert_close(pose_body.translation.z, h_ground, 1e-2);

        let rotation_to_horizontal =
            pose_body
                .rotation
                .rotation_to(&UnitQuaternion::from_axis_angle(
                    &Vector3::y_axis(),
                    PI / 2.0,
                ));
        // rod lying flat on the ground
        assert_close(rotation_to_horizontal.angle(), 0.0, 1e-5);

        let v_final = &v[v.len() - 1][0];
        let v_body = v_final.spatial();
        let v_body_linear = pose_body.rotation * v_body.linear;
        // rod is not moving
        assert_close!(v_body_linear.z, 0.0, 1e-2);
        // TODO: v_x is rather large, and does not seem to decrease with time
        assert_close!(v_body_linear.x, 0.0, 1e-2);
        assert_eq!(v_body.linear.y, 0.0);
        assert_close!(v_body.angular.norm(), 0.0, 1e-3);

        // rod bottom is still at origin.
        // TODO: This is failing by a large margin; make this work
        assert_close!(pose_body.translation.x, 0.0, 1e-2);
    }
}
