use na::dvector;
use wasm_bindgen::prelude::wasm_bindgen;
use web_sys::js_sys::{Float32Array, Uint32Array};

use crate::flog;
use crate::hybrid::{Deformable, Rigid};
use crate::na::vector;
use crate::types::Float;
use crate::{hybrid::Hybrid, spatial::pose::Pose, toJsFloat32Array};

#[wasm_bindgen]
pub struct InterfaceHybrid {
    pub(crate) inner: Hybrid,
}

#[wasm_bindgen]
impl InterfaceHybrid {
    pub fn n_rigid_bodies(&self) -> usize {
        self.inner.rigid_bodies.len()
    }

    pub fn rigid_body_poses(&self) -> Float32Array {
        let mut q = vec![];
        for rigid in self.inner.rigid_bodies.iter() {
            q.extend(rigid.pose.to_array());
        }
        toJsFloat32Array!(q)
    }

    pub fn deformable_nodes(&self) -> Float32Array {
        let mut q: Vec<Float> = vec![];
        for deformable in self.inner.deformables.iter() {
            q.extend(deformable.q.iter());
        }
        toJsFloat32Array!(q)
    }

    /// dofs of each deformable as an array
    pub fn deformable_dofs(&self) -> Uint32Array {
        let dofs: Vec<usize> = self.inner.deformables.iter().map(|x| x.q.len()).collect();
        Uint32Array::from(
            dofs.iter()
                .map(|x| *x as u32)
                .collect::<Vec<u32>>()
                .as_slice(),
        )
    }

    pub fn deformable_faces(&self) -> Uint32Array {
        let mut faces: Vec<usize> = vec![];
        for deformable in self.inner.deformables.iter() {
            faces.extend(
                deformable
                    .faces
                    .iter()
                    .flat_map(|f| *f)
                    .collect::<Vec<usize>>(),
            );
        }
        Uint32Array::from(
            faces
                .iter()
                .map(|x| *x as u32)
                .collect::<Vec<u32>>()
                .as_slice(),
        )
    }

    pub fn deformable_face_ns(&self) -> Uint32Array {
        let face_ns: Vec<usize> = self
            .inner
            .deformables
            .iter()
            .map(|x| x.faces.len())
            .collect();
        Uint32Array::from(
            face_ns
                .iter()
                .map(|x| *x as u32)
                .collect::<Vec<u32>>()
                .as_slice(),
        )
    }

    pub fn step(&mut self, dt: Float) {
        let n_substep = 10;
        let dt = dt / (n_substep as Float);
        for _ in 0..n_substep {
            self.inner.step(dt);
        }
    }
}

#[wasm_bindgen]
pub async fn createHybridSphereAndTetra() -> InterfaceHybrid {
    let mut state = Hybrid::new_canonical();
    state.set_rigid_poses(vec![Pose::translation(vector![2.5, 0., 0.])]);
    let v_rigid = vector![-1., 0., 0.];
    state.set_rigid_velocities(vec![v_rigid]);

    let v = vector![0.25, 0., 0.];
    let v = vec![v, v, v, v];
    state.set_deformable_velocities(vec![v]);

    InterfaceHybrid { inner: state }
}

#[wasm_bindgen]
pub async fn createHybridCube() -> InterfaceHybrid {
    let mut state = Hybrid::empty();
    state.add_rigid(Rigid::new_sphere());
    state.add_rigid(Rigid::new_sphere());
    state.set_rigid_poses(vec![
        Pose::translation(vector![2.5, 0., 0.]),
        Pose::translation(vector![-1.5, 0., 0.]),
    ]);
    state.set_rigid_velocities(vec![vector![-1., 0., 0.], vector![1., 0., 0.]]);

    state.add_deformable(Deformable::new_cube());
    // let v = vector![1. / 7., 0., 0.];
    // let v = vec![v; 7];
    // state.set_deformable_velocities(vec![v]);

    InterfaceHybrid { inner: state }
}
