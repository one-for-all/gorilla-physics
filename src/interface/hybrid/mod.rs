use na::Vector3;
use wasm_bindgen::prelude::wasm_bindgen;
use web_sys::js_sys::{Float32Array, Uint32Array};

use crate::collision::halfspace;
use crate::hybrid::articulated::Articulated;
use crate::hybrid::builders::build_gripper;
use crate::hybrid::visual::Visual;
use crate::hybrid::{Deformable, Rigid};
use crate::interface::cart;
use crate::joint::floating::FloatingJoint;
use crate::joint::{Joint, JointVelocity};
use crate::na::vector;
use crate::spatial::transform::Transform3D;
use crate::types::Float;
use crate::{hybrid::Hybrid, spatial::pose::Pose, toJsFloat32Array};
use crate::{PI, WORLD_FRAME};

#[wasm_bindgen]
pub struct InterfaceHybrid {
    pub(crate) inner: Hybrid,
}

#[wasm_bindgen]
impl InterfaceHybrid {
    pub fn n_rigid_bodies(&self) -> usize {
        self.inner.rigid_bodies.len()
    }

    pub fn n_articulated(&self) -> usize {
        self.inner.articulated.len()
    }

    /// number of bodies in articulated i
    pub fn n_bodies_articulated(&self, i: usize) -> usize {
        self.inner.articulated[i].bodies.len()
    }

    pub fn n_visuals_articulated_body(&self, i: usize, j: usize) -> usize {
        self.inner.articulated[i].bodies[j].visual.len()
    }

    pub fn visual_type(&self, i: usize, j: usize, k: usize) -> usize {
        let visual = &self.inner.articulated[i].bodies[j].visual[k].0;
        return match visual {
            Visual::Sphere(_) => 0,
            Visual::Cuboid(_) => 1,
        };
    }

    pub fn visual_sphere_r(&self, i: usize, j: usize, k: usize) -> Float {
        let visual = &self.inner.articulated[i].bodies[j].visual[k].0;
        return match visual {
            Visual::Sphere(sphere) => sphere.r,
            Visual::Cuboid(_) => panic!("visual is not a sphere"),
        };
    }

    pub fn visual_cuboid_wdh(&self, i: usize, j: usize, k: usize) -> Float32Array {
        let visual = &self.inner.articulated[i].bodies[j].visual[k].0;
        return match visual {
            Visual::Sphere(_) => panic!("visual is not a cuboid"),
            Visual::Cuboid(cuboid) => toJsFloat32Array!([cuboid.w, cuboid.d, cuboid.h]),
        };
    }

    pub fn iso_visual_to_body(&self, i: usize, j: usize, k: usize) -> Float32Array {
        let iso = &self.inner.articulated[i].bodies[j].visual[k].1;
        let mut q: Vec<Float> = vec![];
        q.extend(iso.rotation.coords.iter());
        q.extend(iso.translation.vector.iter());
        toJsFloat32Array!(q)
    }

    pub fn frame_articulated_body(&self, i: usize, j: usize) -> String {
        self.inner.articulated[i].bodies[j].inertia.frame.clone()
    }

    pub fn pose_articulated_body(&self, i: usize, j: usize) -> Float32Array {
        let body = &self.inner.articulated[i].bodies[j];
        toJsFloat32Array!(body.pose.to_array())
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

    /// Vecs of 4-element (normal, distance)
    pub fn halfspaces(&self) -> Float32Array {
        let mut q = vec![];
        for halfspace in self.inner.halfspaces.iter() {
            let n = &halfspace.normal;
            q.extend([n.x, n.y, n.z, -halfspace.distance(&Vector3::zeros())]);
        }
        toJsFloat32Array!(q)
    }

    pub fn step(&mut self, dt: Float) {
        let n_substep = 2;
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

    state.add_deformable(Deformable::new_cube(1e2));
    // let v = vector![1. / 7., 0., 0.];
    // let v = vec![v; 7];
    // state.set_deformable_velocities(vec![v]);

    InterfaceHybrid { inner: state }
}

#[wasm_bindgen]
pub async fn createPendulum() -> InterfaceHybrid {
    let mut state = Hybrid::empty();
    let l = 1.0;
    let r = 0.1;
    let bodies = vec![Rigid::new_sphere_at(
        &vector![-l, 0., 0.],
        1.,
        r,
        "pendulum",
    )];
    let pendulum_to_world = Transform3D::move_x("pendulum", WORLD_FRAME, -1.0);

    let joints = vec![Joint::new_revolute(pendulum_to_world, Vector3::y_axis())];
    state.add_articulated(Articulated::new(bodies, joints));

    state.add_deformable(Deformable::new_cube(1e2));

    InterfaceHybrid { inner: state }
}

#[wasm_bindgen]
pub async fn createDoublePendulumAndCube() -> InterfaceHybrid {
    let mut state = Hybrid::empty();
    let l = 1.0;
    let r = 0.1;
    let pendulum_frame = "pendulum";
    let pendulum2_frame = "pendulum2";
    let bodies = vec![
        Rigid::new_sphere_at(&vector![-l, 0., 0.], 1., r, pendulum_frame),
        Rigid::new_sphere_at(&vector![0., 0., l], 1., r, pendulum2_frame),
    ];
    let pendulum_to_world = Transform3D::move_x(pendulum_frame, WORLD_FRAME, -l);
    let pendulum2_to_pendulum = Transform3D::move_x(pendulum2_frame, pendulum_frame, -l);

    let joints = vec![
        Joint::new_revolute(pendulum_to_world, Vector3::y_axis()),
        Joint::new_revolute(pendulum2_to_pendulum, Vector3::y_axis()),
    ];
    state.add_articulated(Articulated::new(bodies, joints));

    state.add_deformable(Deformable::new_cube(1e2));

    InterfaceHybrid { inner: state }
}

#[wasm_bindgen]
pub async fn createSphereCart() -> InterfaceHybrid {
    let mut state = Hybrid::empty();

    let r = 0.1;
    let cart_frame = "cart";
    let bodies = vec![Rigid::new_sphere_at(
        &vector![0., 0., 0.],
        1.,
        r,
        cart_frame,
    )];
    let cart_to_world = Transform3D::move_x(cart_frame, WORLD_FRAME, 2.5);
    let joints = vec![Joint::new_prismatic(cart_to_world, Vector3::x_axis())];
    let mut articulated = Articulated::new(bodies, joints);
    articulated.set_joint_v(0, JointVelocity::Float(-1.));

    state.add_articulated(articulated);

    state.add_deformable(Deformable::new_cube(1e2));
    // let v = vector![1. / 8., 0., 0.];
    // let v = vec![v; 8];
    // state.set_deformable_velocities(vec![v]);

    InterfaceHybrid { inner: state }
}

#[wasm_bindgen]
pub async fn createCuboidCart() -> InterfaceHybrid {
    let mut state = Hybrid::empty();

    let l_cube = 1.0;
    let cart_offset = 0.01;

    let m = 1.0;
    let w = 1.0;
    let d = 0.1;
    let cart_frame = "cart";
    let cart = Rigid::new_cuboid_at(&vector![0., 0., 0.], m, w, d, d, cart_frame);
    let cart_to_world = Transform3D::move_xyz(
        cart_frame,
        WORLD_FRAME,
        l_cube + 0.5 + w / 2.,
        cart_offset,
        0.0,
    );

    let bodies = vec![cart];
    let joints = vec![Joint::new_prismatic(cart_to_world, Vector3::x_axis())];
    let mut articulated = Articulated::new(bodies, joints);
    articulated.set_joint_v(0, JointVelocity::Float(-1.0));
    state.add_articulated(articulated);

    let cart2_frame = "cart2";
    let cart2 = Rigid::new_cuboid_at(&vector![0., 0., 0.], m, w, d, d, cart2_frame);
    let cart2_to_world =
        Transform3D::move_xyz(cart2_frame, WORLD_FRAME, -0.5 - w / 2., cart_offset, 0.0);
    let bodies = vec![cart2];
    let joints = vec![Joint::new_prismatic(cart2_to_world, Vector3::x_axis())];
    let mut articulated = Articulated::new(bodies, joints);
    articulated.set_joint_v(0, JointVelocity::Float(1.0));
    state.add_articulated(articulated);

    state.add_deformable(Deformable::new_cube(1e2));
    // let v = vector![1. / 8., 0., 0.];
    // let v = vec![v; 8];
    // state.set_deformable_velocities(vec![v]);

    InterfaceHybrid { inner: state }
}

#[wasm_bindgen]
pub async fn createGripperHybrid() -> InterfaceHybrid {
    let state = build_gripper();

    InterfaceHybrid { inner: state }
}
