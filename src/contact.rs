use std::collections::HashMap;
use std::ops::Mul;

use itertools::izip;
use na::Vector3;

use crate::{
    mechanism::MechanismState, spatial_force::Wrench, transform::Transform3D, twist::Twist,
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
    bodies_to_root: &HashMap<u32, Transform3D>,
    twists: &HashMap<u32, Twist>,
) -> HashMap<u32, Wrench> {
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
/// Ref:
///     1. Coeﬀicient of restitution interpreted as damping in vibroimpact,
///         K. H. Hunt and F. R. E. Crossley, 1975
///     2. A Compliant Contact Model with Nonlinear Damping for Simulation of Robotic Systems,
///         D. W. Marhefka and D. E. Orin, 1999
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
    let f = (λ * zn * z_dot + k * zn).max(0.0);

    f * normal
}

#[cfg(test)]
mod contact_tests {
    use na::{dvector, vector, Matrix3, Matrix4};

    use crate::{helpers::build_pendulum, simulate::simulate, util::assert_close, PI};

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
        let (qs, _vs) = simulate(&mut state, final_time, dt, |_state| dvector![0.0]);

        // Assert
        let q_final = qs[qs.len() - 1][0];
        let q_expect = 30.0 * PI / 180.0; // 30 degrees
        assert_close(&dvector![q_final], &dvector![q_expect], 1e-3);
    }
}
