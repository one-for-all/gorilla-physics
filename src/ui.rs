use na::{dvector, vector, Matrix3, Matrix4};
use wasm_bindgen::prelude::*;
use web_sys;

use crate::{
    inertia::SpatialInertia, joint::RevoluteJoint, mechanism::MechanismState,
    rigid_body::RigidBody, simulate::simulate, transform::Transform3D, types::Float,
};

// Helper function to log to the console
fn console_log(message: &str) {
    web_sys::console::log_1(&message.into());
}

#[wasm_bindgen]
extern "C" {
    fn alert(s: &str);
}

#[wasm_bindgen]
pub fn run() {
    let m = 5.0; // Mass of rod
    let l: Float = 7.0; // Length of rod

    let moment_x = 0.0;
    let moment_y = 1.0 / 3.0 * m * l * l;
    let moment_z = 1.0 / 3.0 * m * l * l;
    let moment = Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]);
    let cross_part = vector![m * l / 2.0, 0.0, 0.0];

    let rod_frame = "rod";
    let world_frame = "world";
    let rod_to_world = Transform3D::new(rod_frame, world_frame, &Matrix4::identity());
    let axis = vector![0.0, 1.0, 0.0];

    let mut state = MechanismState {
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

    // Simulate
    let final_time = 10.0;
    let dt = 0.02;

    let (qs, vs) = simulate(&mut state, final_time, dt);
    let data = qs.iter().map(|q| q[0]).collect::<Vec<Float>>();

    let message = format!("qs: {:?}", data);
    alert(&message);
    // console_log(&format!("qs: {:?}", qs));
}
