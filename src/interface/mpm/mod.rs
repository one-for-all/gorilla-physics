use wasm_bindgen::prelude::wasm_bindgen;

use crate::{
    mpm::MPMDeformable,
    types::{Float, FloatArray},
};

#[wasm_bindgen]
pub struct InterfaceMPMDeformable {
    pub(crate) inner: MPMDeformable,
}

#[wasm_bindgen]
impl InterfaceMPMDeformable {
    // Return particle positions as flattened
    pub fn particles(&self) -> FloatArray {
        FloatArray::from(
            self.inner
                .particles
                .iter()
                .flat_map(|p| [p.pos[0], p.pos[1]])
                .collect::<Vec<Float>>()
                .as_slice(),
        )
    }

    pub fn step(&mut self, dt: Float) {
        let n_substeps = 100;
        let dt = dt / n_substeps as Float;
        for _ in 0..n_substeps {
            self.inner.step(dt);
        }
    }
}

#[wasm_bindgen]
pub async fn createMPMDeformable() -> InterfaceMPMDeformable {
    InterfaceMPMDeformable {
        inner: MPMDeformable::new(),
    }
}
