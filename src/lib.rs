#![allow(non_snake_case)]

use types::Float;
pub extern crate nalgebra as na;

pub mod builders;
pub mod collision;
pub mod contact;
pub mod control;
pub mod double_pendulum;
pub mod dynamics;
pub mod energy;
pub mod fem_deformable;
pub mod gpu;
pub mod inertia;
pub mod integrators;
pub mod joint;
pub mod mass_spring_deformable;
pub mod mechanism;
pub mod momentum;
pub mod rigid_body;
pub mod simulate;
pub mod spatial;
pub mod types;
pub mod util;

#[cfg(not(target_arch = "wasm32"))]
pub mod plot;

// Wasm bindings
// #[cfg(target_arch = "wasm32")] // TODO: configure the rust-analyzer to
// include this even with this cfg
pub mod interface;

pub mod helpers;

pub const GRAVITY: Float = 9.81;

pub const PI: Float = std::f32::consts::PI;
pub const TWO_PI: Float = 2.0 * PI;

pub const WORLD_FRAME: &str = "world";
