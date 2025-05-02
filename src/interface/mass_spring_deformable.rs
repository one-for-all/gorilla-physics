use na::DVector;
use wasm_bindgen::{prelude::wasm_bindgen, JsCast};
use wasm_bindgen_futures::JsFuture;
use web_sys::{
    js_sys::{Float32Array, Uint32Array},
    window, Response,
};

use crate::{mass_spring_deformable::MassSpringDeformable, mesh::read_mesh, types::Float};

use super::to_js_uint_array;

#[wasm_bindgen]
pub struct InterfaceMassSpringDeformable {
    pub(crate) inner: MassSpringDeformable,
}

#[wasm_bindgen]
impl InterfaceMassSpringDeformable {
    // Return vertices as flattened
    pub fn vertices(&self) -> Float32Array {
        Float32Array::from(self.inner.q.as_slice().to_vec().as_slice())
    }

    // Return edges as flattened
    pub fn edges(&self) -> Uint32Array {
        Uint32Array::from(
            self.inner
                .edges
                .iter()
                .flat_map(|e| [e.0 as u32, e.1 as u32])
                .collect::<Vec<u32>>()
                .as_slice(),
        )
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

    // Return tetrahedra as flattened
    pub fn tetrahedra(&self) -> Uint32Array {
        to_js_uint_array(&self.inner.tetrahedra)
    }

    pub fn step(&mut self, dt: Float, tau: Vec<Float>) {
        let tau = DVector::from(tau);

        let n_substep = 10;
        for _ in 0..n_substep {
            self.inner.step(dt / (n_substep as Float), &tau);
        }
    }
}

#[wasm_bindgen]
pub async fn createMassSpringBunny() -> InterfaceMassSpringDeformable {
    let window = window().expect("no global `window` exists");
    let resp_value = JsFuture::from(window.fetch_with_str("coarse_bunny.mesh"))
        .await
        .unwrap();
    let resp: Response = resp_value.dyn_into().unwrap();
    let text = JsFuture::from(resp.text().unwrap()).await.unwrap();
    let text_str = text.as_string().unwrap();

    let (vertices, tetrahedra) = read_mesh(&text_str);
    let n_vertices = vertices.len() as Float;
    let mut bunny = MassSpringDeformable::new(vertices, tetrahedra, 1.0 * n_vertices, 1e5);
    bunny.extract_boundary_facets();

    InterfaceMassSpringDeformable { inner: bunny }
}
