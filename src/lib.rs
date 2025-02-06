#![allow(non_snake_case)]

use types::Float;
pub extern crate nalgebra as na;

pub mod contact;
pub mod control;
pub mod double_pendulum;
pub mod dynamics;
pub mod energy;
pub mod geometric_jacobian;
pub mod inertia;
pub mod joint;
pub mod mechanism;
pub mod momentum;
pub mod pose;
pub mod rigid_body;
pub mod simulate;
pub mod spatial_acceleration;
pub mod spatial_force;
pub mod spatial_vector;
pub mod transform;
pub mod twist;
pub mod types;
pub mod util;

// Wasm bindings
pub mod interface;

pub mod helpers;

pub const GRAVITY: Float = 9.81;

pub const PI: Float = std::f32::consts::PI;
pub const TWO_PI: Float = 2.0 * PI;
