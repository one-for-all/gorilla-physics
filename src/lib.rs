#![allow(non_snake_case)]

use types::Float;
pub extern crate nalgebra as na;

pub mod dynamics;
pub mod geometric_jacobian;
pub mod inertia;
pub mod joint;
pub mod mechanism;
pub mod momentum;
pub mod rigid_body;
pub mod simulate;
pub mod spatial_acceleration;
pub mod spatial_force;
pub mod transform;
pub mod twist;
pub mod types;
pub mod util;

#[cfg(test)]
pub mod test_helpers;

const GRAVITY: Float = 9.81;

const PI: Float = std::f32::consts::PI;
