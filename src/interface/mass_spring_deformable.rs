use na::DVector;
use wasm_bindgen::prelude::wasm_bindgen;
use web_sys::js_sys::{Float32Array, Uint32Array};

use crate::{
    collision::mesh::read_mesh, mass_spring_deformable::MassSpringDeformable, types::Float,
};

use super::{to_js_uint_array, util::read_web_file};

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
    let buf = read_web_file("coarse_bunny.mesh").await;

    let (vertices, tetrahedra) = read_mesh(&buf);
    let n_vertices = vertices.len() as Float;
    let mut bunny = MassSpringDeformable::new(vertices, tetrahedra, 1.0 * n_vertices, 1e5);
    bunny.extract_boundary_facets();

    InterfaceMassSpringDeformable { inner: bunny }
}
