use wasm_bindgen::prelude::wasm_bindgen;

use crate::{
    fluid::Fluid2D,
    types::{Float, FloatArray},
};

#[wasm_bindgen]
pub struct InterfaceFluid2D {
    pub(crate) inner: Fluid2D,
}

#[wasm_bindgen]
impl InterfaceFluid2D {
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
}

#[wasm_bindgen]
pub async fn createFluid2D() -> InterfaceFluid2D {
    InterfaceFluid2D {
        inner: Fluid2D::new(),
    }
}
