use std::collections::HashMap;
use std::ops::Mul;

use itertools::izip;
use na::{vector, zero, Vector3};

use crate::{
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
        (self.location - halfspace.point).dot(&halfspace.normal) <= 0.0
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
}

/// Compute the contact wrenches due to contacts
pub fn contact_dynamics(
    state: &MechanismState,
    bodies_to_root: &HashMap<usize, Transform3D>,
    twists: &HashMap<usize, Twist>,
) -> HashMap<usize, Wrench> {
    let mut contact_wrenches = HashMap::new();
    for (jointid, body) in izip!(state.treejointids.iter(), state.bodies.iter()) {
        let bodyid = jointid;
        let mut wrench = Wrench::zero("world");
        let contact_points = &body.contact_points;
        if !contact_points.is_empty() {
            let body_to_root = bodies_to_root.get(bodyid).unwrap();
            let twist = twists.get(bodyid).unwrap();
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
        }
        contact_wrenches.insert(*bodyid, wrench);
    }

    contact_wrenches
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
            let μ = 1.0; // coefficient of friction
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
        helpers::{build_cube, build_rimless_wheel},
        joint::{JointPosition, JointVelocity, ToJointTorqueVec},
        pose::Pose,
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
}
