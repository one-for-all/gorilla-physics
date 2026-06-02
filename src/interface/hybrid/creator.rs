use wasm_bindgen::prelude::wasm_bindgen;

use crate::hybrid::articulated::Articulated;
use crate::hybrid::builders::{build_parallel_bar, build_range_constrained_joint};
use crate::hybrid::{Hybrid, Rigid};
use crate::joint::Joint;
use crate::spatial::transform::Transform3D;
use crate::WORLD_FRAME;
use crate::{hybrid::builders::import_static_body, interface::hybrid::InterfaceHybrid};
use na::{vector, Vector3};

#[wasm_bindgen]
pub async fn createDesktop() -> InterfaceHybrid {
    let mut state = Hybrid::empty();
    state.add_static_body(import_static_body("table/table.obj").await);
    state.add_static_body(import_static_body("soundbox/soundbox.obj").await);

    // Add sphere
    let sphere = Articulated::new_sphere_at("sphere", 1.0, 0.1, &vector![0., 0., 1.2]);
    state.add_articulated(sphere);

    // Add point
    let point = Articulated::new_point_at("point", 1.0, &vector![0.5, 0., 1.2]);
    state.add_articulated(point);

    InterfaceHybrid { inner: state }
}

#[wasm_bindgen]
pub async fn createParallelBar() -> InterfaceHybrid {
    // let state = build_parallel_bar();
    let state = build_range_constrained_joint();

    InterfaceHybrid { inner: state }
}
