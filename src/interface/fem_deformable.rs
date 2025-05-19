use na::DVector;
use wasm_bindgen::{prelude::wasm_bindgen, JsCast};
use wasm_bindgen_futures::JsFuture;
use web_sys::{
    js_sys::{Float32Array, Uint32Array},
    window, Response,
};

use crate::{fem_deformable::FEMDeformable, mesh::read_mesh, types::Float};

#[wasm_bindgen]
pub struct InterfaceFEMDeformable {
    pub(crate) inner: FEMDeformable,
}

#[wasm_bindgen]
impl InterfaceFEMDeformable {
    // Return vertices as flattened
    pub fn vertices(&self) -> Float32Array {
        Float32Array::from(self.inner.q.as_slice().to_vec().as_slice())
    }

    // Return facets as flattened
    pub fn facets(&self) -> Uint32Array {
        Uint32Array::from(
            self.inner
                .boundary_facets
                .iter()
                .flat_map(|f| [f[0] as u32, f[1] as u32, f[2] as u32])
                .collect::<Vec<u32>>()
                .as_slice(),
        )
    }

    pub fn step(&mut self, dt: Float, tau: Vec<Float>) {
        let tau = DVector::from(tau);

        let n_substep = 1;
        for _ in 0..n_substep {
            self.inner.step(dt / (n_substep as Float), &tau);
        }
    }
}

#[wasm_bindgen]
pub async fn createFEMBox() -> InterfaceFEMDeformable {
    let window = window().expect("no global `window` exists");
    let resp_value = JsFuture::from(window.fetch_with_str("box.mesh"))
        .await
        .unwrap();
    let resp: Response = resp_value.dyn_into().unwrap();
    let text = JsFuture::from(resp.text().unwrap()).await.unwrap();
    let text_str = text.as_string().unwrap();

    let (vertices, tetrahedra) = read_mesh(&text_str);
    let mut deformable = FEMDeformable::new(vertices, tetrahedra, 100.0, 6e5, 0.4);
    deformable.extract_boundary_facets();

    InterfaceFEMDeformable { inner: deformable }
}
