use na::{dvector, vector, Matrix3, Matrix4};
use wasm_bindgen::prelude::*;
use web_sys::js_sys;

use crate::{
    control::pendulum_swing_up_and_balance, inertia::SpatialInertia, joint::RevoluteJoint,
    mechanism::MechanismState, rigid_body::RigidBody, simulate::step, transform::Transform3D,
    types::Float, GRAVITY,
};

#[wasm_bindgen]
extern "C" {
    fn alert(s: &str);
}

/// WebAssembly interface to the MechanismState struct.
#[wasm_bindgen]
pub struct InterfaceMechanismState(pub(crate) MechanismState);

#[wasm_bindgen]
impl InterfaceMechanismState {
    #[wasm_bindgen]
    pub fn step(&mut self, dt: Float) -> js_sys::Float32Array {
        let torque = pendulum_swing_up_and_balance(&self.0);

        let (q, _v) = step(&mut self.0, dt, &torque);

        // Convert to a format that Javascript can take
        let q_js = js_sys::Float32Array::new_with_length(q.len() as u32);
        for (i, v) in q.iter().enumerate() {
            q_js.set_index(i as u32, *v);
        }

        q_js
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

    let mut state =
        crate::helpers::build_rod_pendulum(&m, &moment, &cross_part, &rod_to_world, &axis);

    let q_init = dvector![0.1];
    let v_init = dvector![0.0];
    state.update(&q_init, &v_init);

    InterfaceMechanismState(state)
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
        treejoints: dvector![RevoluteJoint {
            init_mat: rod_to_world.mat.clone(),
            transform: rod_to_world,
            axis
        }],
        treejointids: dvector![1],
        bodies: dvector![RigidBody {
            inertia: SpatialInertia {
                frame: rod_frame.to_string(),
                moment,
                cross_part,
                mass: m
            }
        }],
        q: dvector![0.0],
        v: dvector![0.0],
    };

    InterfaceMechanismState(state)
}
