use na::{vector, DVector, UnitVector3};
use wasm_bindgen::prelude::wasm_bindgen;
use web_sys::js_sys::Uint32Array;

use crate::{
    collision::{halfspace::HalfSpace, mesh::read_mesh},
    fem::deformable::FEMDeformable,
    interface::util::read_web_file,
    types::{Float, FloatArray},
};

#[wasm_bindgen]
pub struct InterfaceFEMDeformable {
    pub(crate) inner: FEMDeformable,
}

#[wasm_bindgen]
impl InterfaceFEMDeformable {
    // Return vertices as flattened
    pub fn vertices(&self) -> FloatArray {
        FloatArray::from(self.inner.q.as_slice().to_vec().as_slice())
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

    pub async fn step(&mut self, dt: Float, tau: Vec<Float>) {
        let tau = DVector::from(tau);

        let n_substep = 2;
        for _ in 0..n_substep {
            self.inner.step(dt / (n_substep as Float), &tau).await;
        }
    }
}

#[wasm_bindgen]
pub async fn createFEMBox() -> InterfaceFEMDeformable {
    let buf = read_web_file("box.mesh").await;

    let (vertices, tetrahedra) = read_mesh(&buf);
    let mut deformable = FEMDeformable::new(vertices, tetrahedra, 100.0, 6e5, 0.4).await;

    let angle = Float::to_radians(10.0);
    let normal = UnitVector3::new_normalize(vector![angle.sin(), 0.0, angle.cos()]);
    let ground = HalfSpace::new(normal, -1.2);
    deformable.add_halfspace(ground);
    deformable.enable_gravity = true;

    deformable.extract_boundary_facets();

    InterfaceFEMDeformable { inner: deformable }
}
