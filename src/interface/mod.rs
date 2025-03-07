use crate::contact::ContactPoint;
use crate::joint::floating::FloatingJoint;
use crate::joint::JointVelocity;
use crate::joint::ToFloatDVec;
use crate::joint::ToJointPositionVec;
use crate::joint::ToJointVelocityVec;
use crate::pose::Pose;
use crate::spatial_vector::SpatialVector;
use crate::transform::compute_bodies_to_root;
use controller::InterfaceController;
use itertools::izip;
use na::Rotation3;
use na::UnitQuaternion;
use na::UnitVector3;
use na::Vector3;
use na::{dvector, vector, Matrix3, Matrix4};
use wasm_bindgen::prelude::*;
use web_sys::js_sys;

use crate::{
    contact::HalfSpace,
    helpers::build_double_pendulum,
    inertia::SpatialInertia,
    joint::{revolute::RevoluteJoint, Joint, JointPosition},
    mechanism::MechanismState,
    rigid_body::RigidBody,
    simulate::step,
    transform::Transform3D,
    types::Float,
    PI,
};

pub mod cart;
pub mod cart_pole;
pub mod controller;
pub mod cube;
pub mod double_pendulum;
pub mod hopper;
pub mod pendulum;
pub mod rimless_wheel;

#[wasm_bindgen]
extern "C" {
    fn alert(s: &str);
}

#[wasm_bindgen]
extern "C" {
    #[wasm_bindgen(js_namespace = performance)]
    fn now() -> Float; // Returns milliseconds since epoch
}

#[wasm_bindgen]
pub struct InterfaceSimulator {
    pub(crate) state: InterfaceMechanismState,
    pub(crate) controller: InterfaceController,
}

#[wasm_bindgen]
impl InterfaceSimulator {
    #[wasm_bindgen(constructor)]
    pub fn new(
        state: InterfaceMechanismState,
        controller: InterfaceController,
    ) -> InterfaceSimulator {
        InterfaceSimulator { state, controller }
    }

    #[wasm_bindgen]
    pub fn step(&mut self, dt: Float) -> js_sys::Float32Array {
        let n_substep = 100;
        let mut q = vec![];
        for _ in 0..n_substep {
            // let torque = self.1 .0.vertical_control(&(self.0).0);
            let torque = self.controller.inner.control(&mut (self.state).inner);
            let (_q, _v) = step(&mut (self.state).inner, dt / (n_substep as Float), &torque);
            q = _q;
        }

        // Convert to a format that Javascript can take
        let q = q.to_float_dvec();
        let q_js = js_sys::Float32Array::new_with_length(q.len() as u32);
        for (i, q) in q.iter().enumerate() {
            q_js.set_index(i as u32, *q);
        }

        q_js
    }

    #[wasm_bindgen]
    pub fn poses(&self) -> js_sys::Float32Array {
        self.state.poses()
    }

    #[wasm_bindgen]
    pub fn contact_positions(&self) -> js_sys::Float32Array {
        self.state.contact_positions()
    }
}

/// WebAssembly interface to the MechanismState struct.
#[wasm_bindgen]
pub struct InterfaceMechanismState {
    pub(crate) inner: MechanismState,
}

#[wasm_bindgen]
impl InterfaceMechanismState {
    /// Get the poses of each body in the system
    pub fn poses(&self) -> js_sys::Float32Array {
        let njoints = self.inner.treejointids.len();

        // TODO: use cached bodies_to_root
        let bodies_to_root = compute_bodies_to_root(&self.inner);

        let mut poses = vec![];
        for jointid in self.inner.treejointids.iter() {
            let body_to_root = bodies_to_root.get(jointid).unwrap().mat;

            let rotation_matrix: Matrix3<Float> = body_to_root.fixed_view::<3, 3>(0, 0).into();
            let rotation = Rotation3::from_matrix(&rotation_matrix);
            let translation = body_to_root.fixed_view::<3, 1>(0, 3);

            let euler = rotation.euler_angles();
            poses.extend_from_slice(&[euler.0, euler.1, euler.2]);
            poses.extend_from_slice(&[translation[0], translation[1], translation[2]]);
        }

        // TODO: extract below into a function
        let q_js = js_sys::Float32Array::new_with_length(6 * njoints as u32);
        for (i, q) in poses.iter().enumerate() {
            q_js.set_index(i as u32, *q);
        }
        q_js
    }

    /// Get the positions of each contact point in the system
    pub fn contact_positions(&self) -> js_sys::Float32Array {
        let bodies_to_root = compute_bodies_to_root(&self.inner);

        let mut positions = vec![];
        for (jointid, body) in izip!(self.inner.treejointids.iter(), self.inner.bodies.iter()) {
            let body_to_root = bodies_to_root.get(jointid).unwrap();

            for contact in body.contact_points.iter() {
                let location_in_body = contact.location;
                let location_in_world = body_to_root.transform_point(&location_in_body);
                positions.extend_from_slice(&[
                    location_in_world[0],
                    location_in_world[1],
                    location_in_world[2],
                ]);
            }
        }

        let q_js = js_sys::Float32Array::new_with_length(positions.len() as u32);
        for (i, q) in positions.iter().enumerate() {
            q_js.set_index(i as u32, *q);
        }
        q_js
    }

    #[wasm_bindgen]
    pub fn addHalfSpace(&mut self, normal: Vec<Float>, distance: Float) {
        let halfspace = HalfSpace::new(
            UnitVector3::new_normalize(vector![normal[0], normal[1], normal[2]]),
            distance,
        );
        self.inner.add_halfspace(&halfspace);
    }
}

#[wasm_bindgen]
pub fn createRodPendulumAtBottom(length: Float) -> InterfaceMechanismState {
    let m = 5.0;
    let l: Float = length;

    let moment_x = 1.0 / 3.0 * m * l * l;
    let moment_y = 1.0 / 3.0 * m * l * l;
    let moment_z = 0.0;
    let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
    let cross_part = vector![0.0, 0.0, -m * l / 2.0];

    let rod_to_world = Matrix4::identity(); // transformation from rod to world frame
    let axis = vector![0.0, 1.0, 0.0]; // axis of joint rotation

    let mut state = crate::helpers::build_pendulum(&m, &moment, &cross_part, &rod_to_world, &axis);

    let q_init = vec![JointPosition::Float(0.1)];
    let v_init = vec![0.0].to_joint_vel_vec();
    state.update(&q_init, &v_init);

    InterfaceMechanismState { inner: state }
}

/// Create a uniform rod pendulum that hangs horizontally to the right.
#[wasm_bindgen]
pub fn createRodPendulum(length: Float) -> InterfaceMechanismState {
    let m = 5.0; // Mass of rod
    let l: Float = length; // Length of rod

    let moment_x = 0.0;
    let moment_y = 1.0 / 3.0 * m * l * l;
    let moment_z = 1.0 / 3.0 * m * l * l;
    let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
    let cross_part = vector![m * l / 2.0, 0.0, 0.0];

    let rod_frame = "rod";
    let world_frame = "world";
    let rod_to_world = Transform3D::new(rod_frame, world_frame, &Matrix4::identity());
    let axis = vector![0.0, 1.0, 0.0];

    let state = MechanismState {
        treejoints: vec![Joint::RevoluteJoint(RevoluteJoint {
            init_mat: rod_to_world.mat.clone(),
            transform: rod_to_world,
            axis,
        })],
        treejointids: dvector![1],
        bodies: dvector![RigidBody::new(SpatialInertia {
            frame: rod_frame.to_string(),
            moment,
            cross_part,
            mass: m
        })],
        q: vec![0.0].to_joint_pos_vec(),
        v: vec![0.0].to_joint_vel_vec(),
        halfspaces: dvector![],
    };

    InterfaceMechanismState { inner: state }
}

#[wasm_bindgen]
pub fn createDoublePendulumHorizontal(length: Float) -> InterfaceMechanismState {
    let m = 5.0;
    let l = length;

    let moment_x = 0.0;
    let moment_y = m * l * l;
    let moment_z = m * l * l;
    let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
    let cross_part = vector![m * l, 0., 0.];

    let rod1_to_world = Matrix4::identity();
    let rod2_to_rod1 = Transform3D::move_x(l);
    let axis = vector![0.0, 1.0, 0.0]; // axis of joint rotation

    let mut state = build_double_pendulum(
        &m,
        &moment,
        &cross_part,
        &rod1_to_world,
        &rod2_to_rod1,
        &axis,
    );

    let q_init = vec![
        JointPosition::Float(-PI / 2.0 + 0.1),
        JointPosition::Float(0.0),
    ];
    let v_init = vec![0.0, 0.0].to_joint_vel_vec();
    state.update(&q_init, &v_init);

    InterfaceMechanismState { inner: state }
}

#[wasm_bindgen]
pub fn createSphere(mass: Float, radius: Float) -> InterfaceMechanismState {
    let m = mass;
    let r = radius;

    let moment_x = 2.0 / 5.0 * m * r * r;
    let moment_y = 2.0 / 5.0 * m * r * r;
    let moment_z = 2.0 / 5.0 * m * r * r;
    let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
    let cross_part = vector![0.0, 0.0, 0.0];

    let ball_frame = "ball";
    let world_frame = "world";
    let ball_to_world = Transform3D::identity(&ball_frame, &world_frame);

    let ball = RigidBody::new(SpatialInertia {
        frame: ball_frame.to_string(),
        moment,
        cross_part,
        mass: m,
    });

    let mut state = MechanismState {
        treejoints: vec![Joint::FloatingJoint(FloatingJoint {
            init_mat: ball_to_world.mat.clone(),
            transform: ball_to_world,
        })],
        treejointids: dvector![1],
        bodies: dvector![ball],
        q: vec![JointPosition::Pose(Pose::identity())],
        v: vec![JointVelocity::Spatial(SpatialVector {
            angular: vector![0.0, 0.0, 0.0],
            linear: vector![0.0, 0.0, 0.0],
        })],
        halfspaces: dvector![],
    };

    let q_init = vec![JointPosition::Pose(Pose {
        rotation: UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.0),
        translation: vector![0.0, 0.0, 1.0],
    })];
    let v_init = vec![JointVelocity::Spatial(SpatialVector {
        angular: vector![0.0, 0.0, 0.0],
        linear: vector![0.0, 0.0, 10.0],
    })];
    state.update(&q_init, &v_init);

    state.add_contact_point(&ContactPoint {
        frame: "ball".to_string(),
        location: vector![0., 0., -r],
    });

    InterfaceMechanismState { inner: state }
}
