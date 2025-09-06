use na::{vector, DVector};
use wasm_bindgen::prelude::wasm_bindgen;
use web_sys::js_sys::{Float32Array, Uint32Array};

use crate::{
    fem::cloth::Cloth,
    types::{Float, FloatArray},
};

#[wasm_bindgen]
pub struct InterfaceCloth {
    pub(crate) inner: Cloth,
}

#[wasm_bindgen]
impl InterfaceCloth {
    pub fn vertices(&self) -> Float32Array {
        Float32Array::from(
            self.inner
                .q
                .iter()
                .map(|qi| *qi as f32)
                .collect::<Vec<f32>>()
                .as_slice(),
        )
    }

    pub fn faces(&self) -> Uint32Array {
        Uint32Array::from(
            self.inner
                .triangles
                .iter()
                .flat_map(|f| [f[0] as u32, f[1] as u32, f[2] as u32])
                .collect::<Vec<u32>>()
                .as_slice(),
        )
    }

    pub fn step(&mut self, dt: Float) {
        let n_substep = 10;
        for _ in 0..n_substep {
            self.inner.step(dt / (n_substep as Float));
        }
    }
}

#[wasm_bindgen]
pub async fn createCloth() -> InterfaceCloth {
    let vertices = vec![
        vector![0., 0., 0.],
        vector![1., 0., 0.],
        vector![0., 1., 0.],
        vector![1., 1., 0.],
        // vector![0., 0., -1.],
        // vector![1., 0., -1.],
    ];
    #[rustfmt::skip]
    let triangles = vec![
        [0, 1, 2],
        [1, 3, 2],
        // [0, 2, 1]
    ];
    let mut cloth = Cloth::new(vertices, triangles);
    // cloth.q[2 * 3 + 2] = -1.01;
    // cloth.q[3 * 3 + 2] = -1.01;

    InterfaceCloth { inner: cloth }
}
