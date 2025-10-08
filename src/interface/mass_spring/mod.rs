use na::{vector, Vector3};
use wasm_bindgen::prelude::wasm_bindgen;
use web_sys::js_sys::{Float32Array, Uint32Array};

use crate::{
    mass_spring::MassSpring,
    types::{Float, FloatArray},
};

#[wasm_bindgen]
pub struct InterfaceMassSpring {
    pub(crate) inner: MassSpring,
}

#[wasm_bindgen]
impl InterfaceMassSpring {
    pub fn nodes(&self) -> Float32Array {
        Float32Array::from(
            self.inner
                .q
                .iter()
                .map(|qi| *qi as f32)
                .collect::<Vec<f32>>()
                .as_slice(),
        ) // TODO: util function that does this tranformation
    }

    pub fn facets(&self) -> Uint32Array {
        Uint32Array::from(
            self.inner
                .faces
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
pub async fn createMassSpringTetrahedron() -> InterfaceMassSpring {
    let mut deformable = MassSpring::unit_tetrahedron();

    let v_linear = vector![0., 0., 5.]; // linear velocity
    let omega = vector![0., 0., 0.]; // angular velocity around com
    let com = deformable.com();
    let v: Vec<Vector3<Float>> = deformable
        .nodes
        .iter()
        .map(|n| omega.cross(&(n - com)) + v_linear)
        .collect();
    deformable.set_velocity(v);

    InterfaceMassSpring { inner: deformable }
}
