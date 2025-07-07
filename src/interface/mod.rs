use crate::contact::ContactPoint;
use crate::control::ControlInput;
use crate::integrators::Integrator;
use crate::joint::floating::FloatingJoint;
use crate::joint::JointVelocity;
use crate::joint::ToFloatDVec;
use crate::joint::ToJointVelocityVec;
use crate::spatial::pose::Pose;
use crate::spatial::spatial_vector::SpatialVector;
use crate::spatial::transform::compute_bodies_to_root;
use crate::WORLD_FRAME;
use controller::InterfaceController;
use itertools::izip;
use na::zero;
use na::DVector;
use na::Isometry3;
use na::UnitQuaternion;
use na::UnitVector3;
use na::Vector3;
use na::{vector, Matrix3};
use wasm_bindgen::prelude::*;
use web_sys::js_sys;
use web_sys::js_sys::Float32Array;
use web_sys::js_sys::Uint32Array;

use crate::{
    contact::HalfSpace,
    helpers::build_double_pendulum,
    inertia::SpatialInertia,
    joint::{revolute::RevoluteJoint, Joint, JointPosition},
    mechanism::MechanismState,
    rigid_body::RigidBody,
    simulate::step,
    spatial::transform::Transform3D,
    types::Float,
    PI,
};

pub mod cart;
pub mod cart_pole;
pub mod controller;
pub mod cube;
pub mod double_pendulum;
pub mod fem_deformable;
pub mod hopper;
pub mod mass_spring_deformable;
pub mod mesh;
pub mod pendulum;
pub mod quadruped;
pub mod rimless_wheel;
pub mod robot_arm;
pub mod util;

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
    pub fn step(&mut self, dt: Float, control_input: Vec<Float>) -> js_sys::Float32Array {
        let input = ControlInput::new(control_input);

        let n_substep = 2;
        let mut q = vec![];
        for _ in 0..n_substep {
            let torque = self.controller.inner.control(&mut (self.state).inner, None); // Some(&input));
            let (_q, _v) = step(
                &mut (self.state).inner,
                dt / (n_substep as Float),
                &torque,
                &Integrator::VelocityStepping,
            );
            q = _q;
        }

        // Convert to a format that Javascript can take
        // TODO: make it into a reusable function
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

    /// Get the vertices of the collider mesh on a body
    #[wasm_bindgen]
    pub fn vertices(&self, body_id: usize) -> js_sys::Float32Array {
        self.state.vertices(body_id)
    }

    /// Get the faces of the collider mesh on a body
    #[wasm_bindgen]
    pub fn facets(&self, body_id: usize) -> Uint32Array {
        self.state.facets(body_id)
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
            let body_to_root = bodies_to_root.get(jointid).unwrap().iso;
            let rotation = body_to_root.rotation;
            let translation = body_to_root.translation.vector;

            let euler = rotation.euler_angles(); // TODO: use a more stable representation? to avoid jumping effects.
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
        self.inner.add_halfspace(halfspace);
    }

    #[wasm_bindgen]
    pub fn addHalfSpaceCustom(
        &mut self,
        normal: Vec<Float>,
        distance: Float,
        alpha: Float,
        mu: Float,
    ) {
        let halfspace = HalfSpace::new_with_params(
            UnitVector3::new_normalize(vector![normal[0], normal[1], normal[2]]),
            distance,
            alpha,
            mu,
        );
        self.inner.add_halfspace(halfspace);
    }

    /// Get the vertices of the collider mesh on a body
    #[wasm_bindgen]
    pub fn vertices(&self, body_id: usize) -> Float32Array {
        let vertices = &self.inner.bodies[body_id]
            .collider
            .as_ref()
            .expect(&format!("body {} should have mesh", body_id))
            .geometry
            .mesh()
            .vertices;
        let q = DVector::from_iterator(
            vertices.len() * 3,
            vertices.iter().flat_map(|v| v.iter().copied()),
        );
        Float32Array::from(q.as_slice().to_vec().as_slice())
    }

    /// Get the vertices of the collider mesh on a body
    #[wasm_bindgen]
    pub fn facets(&self, body_id: usize) -> Uint32Array {
        let facets = &self.inner.bodies[body_id]
            .collider
            .as_ref()
            .expect(&format!("body {} should have mesh", body_id))
            .geometry
            .mesh()
            .faces;
        Uint32Array::from(
            facets
                .iter()
                .flat_map(|f| [f[0] as u32, f[1] as u32, f[2] as u32])
                .collect::<Vec<u32>>()
                .as_slice(),
        )
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

    let rod_to_world = Isometry3::identity(); // transformation from rod to world frame
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
    let rod_to_world = Transform3D::identity(rod_frame, world_frame);
    let axis = vector![0.0, 1.0, 0.0];

    let treejoints = vec![Joint::RevoluteJoint(RevoluteJoint::new(rod_to_world, axis))];
    let bodies = vec![RigidBody::new(SpatialInertia {
        frame: rod_frame.to_string(),
        moment,
        cross_part,
        mass: m,
    })];
    let state = MechanismState::new(treejoints, bodies);

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

    let rod1_to_world = Isometry3::identity();
    let rod2_to_rod1 = Isometry3::translation(l, 0., 0.);
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

    let treejoints = vec![Joint::FloatingJoint(FloatingJoint {
        init_iso: ball_to_world.iso,
        transform: ball_to_world,
    })];
    let bodies = vec![ball];
    let mut state = MechanismState::new(treejoints, bodies);

    let q_init = vec![JointPosition::Pose(Pose {
        rotation: UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.0),
        translation: vector![0.0, 0.0, 1.0],
    })];
    let v_init = vec![JointVelocity::Spatial(SpatialVector {
        angular: vector![0.0, 0.0, 0.0],
        linear: vector![0.0, 0.0, 10.0],
    })];
    state.update(&q_init, &v_init);

    state.add_contact_point(ContactPoint::new("ball", vector![0., 0., -r]));

    InterfaceMechanismState { inner: state }
}

#[wasm_bindgen]
pub fn createCompassGait() -> InterfaceMechanismState {
    let m_hip = 10.0;
    let r_hip = 0.3;
    let m_leg = 5.0;
    let l_leg = 1.0;

    let hip_frame = "hip";
    let moment_x = m_hip * r_hip * r_hip * 2.0 / 5.0;
    let hip = RigidBody::new(SpatialInertia {
        frame: hip_frame.to_string(),
        moment: Matrix3::from_diagonal(&vector![moment_x, moment_x, moment_x]),
        cross_part: zero(),
        mass: m_hip,
    });

    let moment_x = m_leg * (l_leg / 2.0) * (l_leg / 2.0);
    let moment_y = moment_x;
    let moment_z = 0.0;
    let cross_part = vector![0., 0., -m_leg * l_leg / 2.0];

    let left_leg_frame = "left leg";
    let left_leg = RigidBody::new(SpatialInertia {
        frame: left_leg_frame.to_string(),
        moment: Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]),
        cross_part,
        mass: m_leg,
    });

    let right_leg_frame = "right leg";
    let right_leg = RigidBody::new(SpatialInertia {
        frame: right_leg_frame.to_string(),
        moment: Matrix3::from_diagonal(&vector![moment_x, moment_y, moment_z]),
        cross_part,
        mass: m_leg,
    });

    let hip_to_world = Transform3D::identity(hip_frame, WORLD_FRAME);
    let left_leg_to_hip = Transform3D::identity(left_leg_frame, hip_frame);
    let left_leg_axis = vector![0., -1., 0.];
    let right_leg_to_hip = Transform3D::identity(right_leg_frame, hip_frame);
    let right_leg_axis = vector![0., -1., 0.];
    let treejoints = vec![
        Joint::FloatingJoint(FloatingJoint::new(hip_to_world)),
        Joint::RevoluteJoint(RevoluteJoint::new(left_leg_to_hip, left_leg_axis)),
        Joint::RevoluteJoint(RevoluteJoint::new(right_leg_to_hip, right_leg_axis)),
    ];

    let bodies = vec![hip, left_leg, right_leg];

    let mut state = MechanismState::new(treejoints, bodies);
    state.add_contact_point(ContactPoint::new(left_leg_frame, vector![0., 0., -l_leg]));
    state.add_contact_point(ContactPoint::new(right_leg_frame, vector![0., 0., -l_leg]));

    state.set_joint_q(
        1,
        JointPosition::Pose(Pose {
            rotation: UnitQuaternion::identity(),
            translation: vector![0., 0., l_leg],
        }),
    );

    state.set_joint_q(2, JointPosition::Float(Float::to_radians(30.0)));
    state.set_joint_q(3, JointPosition::Float(Float::to_radians(-30.0)));

    InterfaceMechanismState { inner: state }
}

fn to_js_float_array(from: &Vec<Vector3<Float>>) -> js_sys::Float32Array {
    let from: Vec<f32> = from.iter().flat_map(|v| [v.x, v.y, v.z]).collect();
    js_sys::Float32Array::from(from.as_slice())
}

fn to_js_uint_array(from: &Vec<Vec<usize>>) -> js_sys::Uint32Array {
    let from: Vec<u32> = from.iter().flatten().map(|v| *v as u32).collect();
    js_sys::Uint32Array::from(from.as_slice())
}
