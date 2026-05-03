use wasm_bindgen::prelude::wasm_bindgen;

use crate::hybrid::articulated::Articulated;
use crate::hybrid::Hybrid;
use crate::{hybrid::builders::import_static_body, interface::hybrid::InterfaceHybrid};
use na::vector;

#[wasm_bindgen]
pub async fn createDesktop() -> InterfaceHybrid {
    let mut state = Hybrid::empty();
    state.add_static_body(import_static_body("table/table.obj").await);

    // Add sphere
    let sphere = Articulated::new_sphere_at("sphere", 1.0, 0.1, &vector![0., 0., 1.2]);
    state.add_articulated(sphere);

    // Add point
    let point = Articulated::new_point_at("point", 1.0, &vector![0.5, 0., 1.2]);
    state.add_articulated(point);

    InterfaceHybrid { inner: state }
}
