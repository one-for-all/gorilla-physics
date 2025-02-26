use std::collections::HashMap;
use std::ops::Mul;

use itertools::izip;
use na::{vector, UnitVector3, Vector3};

use crate::WORLD_FRAME;
use crate::{
    control::energy_control::spring,
    mechanism::MechanismState,
    spatial_force::Wrench,
    transform::{compute_bodies_to_root, Transform3D},
    twist::Twist,
    types::Float,
};

#[derive(Clone, PartialEq, Debug)]
pub struct ContactPoint {
    pub frame: String, // the frame the contact point is expressed in
    pub location: Vector3<Float>,
}

impl ContactPoint {
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
        }
    }

    pub fn inside_halfspace(&self, halfspace: &HalfSpace) -> bool {
        halfspace.has_inside(&self.location)
    }

    pub fn compute_contact(&self, halfspace: &HalfSpace) -> (Float, Vector3<Float>) {
        let penetration = -(self.location - halfspace.point).dot(&halfspace.normal);
        let normal = halfspace.normal;
        (penetration, normal)
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct HalfSpace {
    pub point: Vector3<Float>,  // A point in the half-space
    pub normal: Vector3<Float>, // Outward normal direction of the half-space
}

impl HalfSpace {
    // Create a half-space that is moved along normal by distance, from origin
    pub fn new(normal: Vector3<Float>, distance: Float) -> Self {
        HalfSpace {
            point: normal * distance,
            normal,
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

        // Handle rigid body contacts
        let contact_points = &body.contact_points;
        for contact_point in contact_points {
            let contact_point = contact_point.transform(body_to_root);
            let velocity = twist.point_velocity(&contact_point);
            for halfspace in state.halfspaces.iter() {
                if !contact_point.inside_halfspace(&halfspace) {
                    continue;
                }
                let (penetration, normal) = contact_point.compute_contact(&halfspace);
                let contact_force = calculate_contact_force(&penetration, &velocity, &normal);
                wrench += Wrench::from_force(&contact_point.location, &contact_force, "world");
            }
        }

        // Handle spring contacts
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
                        spring(spring_contact.l_rest, direction_distance, spring_contact.k);
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

    contact_wrenches
}

    false
}

/// Calculate the contact force expressed in world frame
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
pub fn calculate_contact_force(
    penetration: &Float,
    velocity: &Vector3<Float>,
    normal: &Vector3<Float>,
) -> Vector3<Float> {
    let z = penetration;
    let z_dot = -velocity.dot(normal);

    // Hunt-Crossley model for normal force
    let zn = z.powf(3.0 / 2.0);
    let k = 50e3;
    let a = 0.9; // how much velocity is lost, default 0.2
                 // coefficient of restitution e ~= 1-a*v_in
    let λ = 3.0 / 2.0 * a * k;
    let π = (λ * zn * z_dot + k * zn).max(0.0);
    let f_normal = π * normal;

    // friction force in tangential direction - regularized coulomb friction
    let v_t = velocity + z_dot * normal; // tangential velocity
    let v_t_norm = v_t.norm();
    let f_friction = {
        if v_t_norm == 0.0 {
            Vector3::zeros()
        } else {
            let v_s = 1e-3; // slip tolerance, amount of velocity allowed for contact that should be stationary
            let s = v_t_norm / v_s;
            let μ = 0.5; // coefficient of friction
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

#[cfg(test)]
mod contact_tests {
    use crate::{
        control::energy_control::Controller,
        helpers::{build_SLIP, build_cube, build_rimless_wheel},
        interface::controller::NullController,
        joint::{JointPosition, JointVelocity, ToJointTorqueVec},
        pose::Pose,
        simulate::step,
        spatial_vector::SpatialVector,
        util::assert_close,
    };
    use na::{dvector, vector, Matrix3, Matrix4, UnitQuaternion};

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
        state.add_contact_point(&ContactPoint {
            frame: "rod".to_string(),
            location: vector![l, 0., 0.],
        });

        state.add_halfspace(&&HalfSpace::new(vector![0., 0., 1.], -5.0));

        // Act
        let final_time = 5.0;
        let dt = 0.001;
        let (qs, _vs) = simulate(&mut state, final_time, dt, |_state| {
            vec![0.0].to_joint_torque_vec()
        });

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
        state.add_halfspace(&HalfSpace::new(vector![0., 0., 1.], h_ground));

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
        let (qs, _vs) = simulate(&mut state, final_time, dt, |_state| vec![]);

        // Assert
        let q_final = qs[qs.len() - 1][0].pose();
        assert_close(&q_final.translation.z, &(h_ground + l / 2.0), 1e-2);
    }

    #[test]
    fn cube_slide_ground() {
        // Arrange
        let m = 3.0;
        let l = 1.0;
        let mut state = build_cube(m, l);

        let h_ground = -l / 2.0;
        state.add_halfspace(&HalfSpace::new(vector![0., 0., 1.], h_ground));

        let q_init = vec![JointPosition::Pose(Pose {
            rotation: UnitQuaternion::identity(),
            translation: vector![0.0, 0.0, 0.0],
        })];
        let v_init = vec![JointVelocity::Spatial(SpatialVector {
            angular: vector![0.0, 0.0, 0.0],
            linear: vector![1.0, 0.0, 0.0],
        })];
        state.update(&q_init, &v_init);

        // Act
        let final_time = 5.0;
        let dt = 1e-3;
        let (qs, vs) = simulate(&mut state, final_time, dt, |_state| vec![]);

        // Assert
        let q_final = qs[qs.len() - 1][0].pose();
        assert_close(&q_final.translation.z, &0.0, 1e-2);

        let v_final = vs[vs.len() - 1][0].spatial();
        assert_close(&v_final.linear.norm(), &0.0, 5e-3);
        assert_close(&v_final.angular.norm(), &0.0, 1e-2);
    }

    /// Hit the ground with rotational and translational velocity
    #[test]
    fn cube_hit_ground() {
        // Arrange
        let m = 3.0;
        let l = 1.0;
        let mut state = build_cube(m, l);

        let h_ground = -10.0;
        state.add_halfspace(&HalfSpace::new(vector![0., 0., 1.], h_ground));

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
        let (qs, vs) = simulate(&mut state, final_time, dt, |_state| vec![]);

        // Assert
        let q_final = qs[qs.len() - 1][0].pose();
        assert_close(&q_final.translation.z, &(h_ground + l / 2.0), 1e-2);

        let v_final = vs[vs.len() - 1][0].spatial();
        assert_close(&v_final.linear.norm(), &0.0, 5e-3);
        assert_close(&v_final.angular.norm(), &0.0, 1e-2);
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
        let normal = vector![angle.sin(), 0.0, angle.cos()];
        let h_ground = -20.0;
        state.add_halfspace(&HalfSpace::new(normal, h_ground));

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
        let (qs, vs) = simulate(&mut state, final_time, dt, |_state| vec![]);

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

    /// Spring Loaded Inverted Pendulum (SLIP)
    #[test]
    fn SLIP() {
        // Arrange
        let m = 0.54;
        let r = 0.1;
        let l_rest = 0.2;
        let angle = Float::to_radians(45.0);
        let k_spring = 500.0;

        let direction = UnitVector3::new_normalize(vector![angle.sin(), 0., -angle.cos()]);
        let mut state = build_SLIP(m, r, l_rest, angle, k_spring);

        let h_ground = -0.3;
        state.add_halfspace(&HalfSpace::new(vector![0., 0., 1.], h_ground));

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
            let torque = controller.control(&mut state);
            let (q, v) = step(&mut state, dt, &torque);

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
            assert_close(&energy, &initial_energy, 1e-2);
        }

        // Hopping height settled
        assert_close(&hs[hs.len() - 3], &hs[hs.len() - 2], 1e-3);
        assert_close(&hs[hs.len() - 2], &hs[hs.len() - 1], 1e-3);
    }
}
