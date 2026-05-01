use wasm_bindgen::prelude::wasm_bindgen;

use crate::{hybrid::builders::build_table, interface::hybrid::InterfaceHybrid};

#[wasm_bindgen]
pub async fn createTable() -> InterfaceHybrid {
    let state = build_table().await;
    InterfaceHybrid { inner: state }
}
