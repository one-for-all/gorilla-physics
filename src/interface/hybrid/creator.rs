use wasm_bindgen::prelude::wasm_bindgen;

use crate::hybrid::articulated::Articulated;
use crate::{hybrid::builders::build_table, interface::hybrid::InterfaceHybrid};
use na::vector;

#[wasm_bindgen]
pub async fn createTable() -> InterfaceHybrid {
    let mut state = build_table().await;

    // Add sphere
    let sphere = Articulated::new_sphere_at("sphere", 1.0, 0.1, &vector![0., 0., 1.2]);
    state.add_articulated(sphere);

    InterfaceHybrid { inner: state }
}
