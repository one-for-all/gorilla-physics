use na::Vector3;
use wasm_bindgen::prelude::wasm_bindgen;
use web_sys::js_sys::{Float32Array, Uint32Array};

use crate::hybrid::articulated::Articulated;
use crate::hybrid::builders::{build_cube_frenzy, build_gripper_cube, build_teddy};
use crate::hybrid::control::{ArticulatedController, NetEvent, NetRequest, NullArticulatedController};
use crate::hybrid::profile::PHASE_NAMES;
use crate::hybrid::{visual::Visual, Deformable, Rigid};
use crate::interface::util::read_web_file;
use crate::joint::{Joint, JointPosition, JointVelocity};
use crate::na::vector;
use crate::spatial::spatial_vector::SpatialVector;
use crate::spatial::transform::Transform3D;
use crate::types::Float;
use crate::{flog, WORLD_FRAME};
use crate::{hybrid::Hybrid, spatial::pose::Pose, toJsFloat32Array, toJsUint32Array};

pub mod creator;

#[wasm_bindgen]
pub struct InterfaceHybrid {
    pub(crate) inner: Hybrid,
    /// The network requests most recently taken off a controller, held so the
    /// accessors below can read them apart. See `net_take_requests_at`.
    net_staged: Vec<NetRequest>,
}

impl InterfaceHybrid {
    pub fn new(hybrid: Hybrid) -> Self {
        console_error_panic_hook::set_once(); // Make panic messages show up in console
        Self {
            inner: hybrid,
            net_staged: Vec::new(),
        }
    }

    /// Mutable access to the wrapped scene, so downstream crates can add bodies
    /// to a simulation that is already running.
    pub fn hybrid_mut(&mut self) -> &mut Hybrid {
        &mut self.inner
    }

    /// The material groups of a mesh visual, resolved against the visual's own
    /// colour. Behind `visual_mesh_groups` and `visual_mesh_group_colors`, which
    /// are two halves of this one answer and must not disagree about how many
    /// groups there are.
    fn mesh_draw_groups(
        &self,
        i: usize,
        j: usize,
        k: usize,
    ) -> Vec<(usize, usize, Vector3<Float>)> {
        let visual = &self.inner.articulated[i].bodies[j].visual[k];
        let fallback = visual.2.unwrap_or(vector![1., 0., 0.]);
        match &visual.0 {
            Visual::RigidMesh(mesh) => mesh.draw_groups(fallback),
            _ => panic!("visual is not a rigid mesh"),
        }
    }
}

#[wasm_bindgen]
impl InterfaceHybrid {
    pub fn reset(&mut self) {
        self.inner.reset();
    }

    pub fn time(&self) -> Float {
        self.inner.time
    }

    /// The names of the phases `profile_take` reports, comma-separated and in
    /// the same order, so a caller never writes that order down a second time.
    pub fn profile_labels(&self) -> String {
        PHASE_NAMES.join(",")
    }

    /// What each phase of `step` has cost since this was last called: the
    /// number of steps counted, then milliseconds per phase in `profile_labels`
    /// order.
    ///
    /// Reading resets, so successive calls carve the run into windows -- poll
    /// it on an interval and each answer describes the interval that just
    /// ended. The step count leads because it is what makes the rest readable:
    /// the same 8 ms means something different over 30 steps than over 3000,
    /// and a window with no steps in it (a paused or backgrounded tab) is all
    /// zeroes rather than a division by zero.
    pub fn profile_take(&mut self) -> Float32Array {
        let (totals, steps) = self.inner.profile.take();
        let mut out = vec![steps as Float];
        out.extend_from_slice(&totals);
        toJsFloat32Array!(out)
    }

    pub fn get_uart(&self) -> String {
        self.inner.controllers[0].get_uart()
    }

    /// UART output of the controller running articulated `i`.
    ///
    /// `get_uart` above answers for the first articulated, which is all a scene
    /// built around one robot needs. A scene that gains robots at runtime has to
    /// say which one it means -- as `reboot_code_controller` already does.
    pub fn get_uart_at(&self, i: usize) -> String {
        self.inner.controllers[i].get_uart()
    }

    pub fn send_uart(&mut self, payload: &str) {
        self.inner.controllers[0].send_uart(payload);
    }

    /// Send `payload` to the controller running articulated `i`, as typing into
    /// a serial monitor does.
    ///
    /// The counterpart of `get_uart_at`, and there for the same reason: a scene
    /// that gains robots at runtime has to say which board it means.
    pub fn send_uart_at(&mut self, i: usize, payload: &str) {
        self.inner.controllers[i].send_uart(payload);
    }

    /// Press or release switch `switch` on the board running articulated `i`.
    ///
    /// The press is held until it is released: the controller drives the pin
    /// from outside the chip, exactly as a finger holding a button down does.
    pub fn set_switch_at(&mut self, i: usize, switch: usize, pressed: bool) {
        self.inner.controllers[i].set_switch(switch, pressed);
    }

    /// Hold joystick `joystick` on the board running articulated `i` at
    /// `x`, `y`, with its cap switch pressed or not.
    ///
    /// The whole state of the stick every time, unlike `set_switch_at`'s one
    /// bit: a pointer dragging a pad reports where it is rather than what
    /// changed, and the controller is what works out which pins that moves.
    pub fn set_joystick_at(
        &mut self,
        i: usize,
        joystick: usize,
        x: Float,
        y: Float,
        pressed: bool,
    ) {
        self.inner.controllers[i].set_joystick(joystick, x, y, pressed);
    }

    /// Push a force onto body `body` of articulated `i`, acting at world point
    /// `p` and held until it is replaced or cleared.
    ///
    /// For a caller outside the physics -- a cursor dragging a link -- which is
    /// why it is a plain force and not a constraint: what the body does with it
    /// is still the solve's answer, so a link held by its servos barely moves
    /// and a loose one swings.
    pub fn set_applied_force_at(
        &mut self,
        i: usize,
        body: usize,
        px: Float,
        py: Float,
        pz: Float,
        fx: Float,
        fy: Float,
        fz: Float,
    ) {
        self.inner.articulated[i].set_applied_force(
            body,
            vector![px, py, pz],
            vector![fx, fy, fz],
        );
    }

    /// Take the force above off articulated `i`. A step does not spend one, so
    /// this is the only thing that ends it.
    pub fn clear_applied_force_at(&mut self, i: usize) {
        self.inner.articulated[i].clear_applied_force();
    }

    /// What the hardware running articulated `i` has to warn about, if
    /// anything. Polled: it depends on what the firmware has done to its pins,
    /// so it changes as the sketch runs rather than at any one moment.
    pub fn hardware_warning_at(&self, i: usize) -> Option<String> {
        self.inner.controllers[i].hardware_warning()
    }

    /// What camera `camera` on the board running articulated `i` wants drawn
    /// for it: `[format, width, height, fps]`, or empty while it wants
    /// nothing. Format is 1 for JPEG, 2 for RGB565, 3 for grayscale.
    ///
    /// Polled, like `hardware_warning_at`: what a sensor is configured to
    /// produce is the running firmware's doing, and changes when it calls
    /// `esp_camera_init` -- or when a new sketch is flashed.
    pub fn camera_request_at(&self, i: usize, camera: usize) -> Vec<u32> {
        use crate::hybrid::control::CameraFrameFormat;
        match self.inner.controllers[i].camera_request(camera) {
            Some(request) => {
                let format = match request.format {
                    CameraFrameFormat::Jpeg => 1,
                    CameraFrameFormat::Rgb565 => 2,
                    CameraFrameFormat::Grayscale => 3,
                };
                vec![format, request.width, request.height, request.fps as u32]
            }
            None => vec![],
        }
    }

    /// Hand camera `camera` on the board running articulated `i` its next
    /// picture, as tightly packed RGBA (`width * height * 4`, top row first)
    /// at the size the current request asked for. The controller packs it
    /// into whatever its sensor puts on the wire; the firmware reads it out
    /// through its own camera driver.
    pub fn supply_camera_frame_at(&mut self, i: usize, camera: usize, rgba: &[u8]) {
        self.inner.controllers[i].supply_camera_frame(camera, rgba);
    }

    /// Take everything the board running articulated `i` wants of the
    /// network, and say how many things that was.
    ///
    /// A request is several values of different types -- a host, a port, a
    /// payload -- so it is read back by the accessors below rather than
    /// returned whole; `wasm_bindgen` has no way to hand over a list of
    /// mixed records, and inventing a wire format for one caller would be a
    /// second thing to keep in step. The requests stay here until the next
    /// call takes their place.
    pub fn net_take_requests_at(&mut self, i: usize) -> usize {
        self.net_staged = self.inner.controllers[i].net_take_requests();
        self.net_staged.len()
    }

    /// Which kind of request `n` is: 0 to open, 1 to send, 2 to close.
    pub fn net_request_kind(&self, n: usize) -> u32 {
        match self.net_staged[n] {
            NetRequest::Open { .. } => 0,
            NetRequest::Send { .. } => 1,
            NetRequest::Close { .. } => 2,
        }
    }

    /// The connection request `n` is about. Ids are never reused, so one the
    /// host has forgotten stays forgotten.
    pub fn net_request_id(&self, n: usize) -> u32 {
        match self.net_staged[n] {
            NetRequest::Open { id, .. } | NetRequest::Send { id, .. } | NetRequest::Close { id } => id,
        }
    }

    /// The host to open, for a request of kind 0.
    pub fn net_request_host(&self, n: usize) -> String {
        match &self.net_staged[n] {
            NetRequest::Open { host, .. } => host.clone(),
            _ => String::new(),
        }
    }

    /// The port to open, for a request of kind 0.
    pub fn net_request_port(&self, n: usize) -> u32 {
        match self.net_staged[n] {
            NetRequest::Open { port, .. } => port as u32,
            _ => 0,
        }
    }

    /// The path to ask for, for a request of kind 0.
    pub fn net_request_path(&self, n: usize) -> String {
        match &self.net_staged[n] {
            NetRequest::Open { path, .. } => path.clone(),
            _ => String::new(),
        }
    }

    /// Whether request `n`'s payload is text rather than binary.
    pub fn net_request_is_text(&self, n: usize) -> bool {
        match self.net_staged[n] {
            NetRequest::Send { text, .. } => text,
            _ => false,
        }
    }

    /// The payload to send, for a request of kind 1.
    pub fn net_request_data(&self, n: usize) -> Vec<u8> {
        match &self.net_staged[n] {
            NetRequest::Send { data, .. } => data.clone(),
            _ => Vec::new(),
        }
    }

    /// Connection `id` on articulated `i` is up. Not to be sent before it
    /// really is: the firmware answers its own handshake on the strength of
    /// this.
    pub fn net_opened_at(&mut self, i: usize, id: u32) {
        self.inner.controllers[i].net_deliver(id, NetEvent::Opened);
    }

    /// A message has arrived on connection `id` of articulated `i`.
    pub fn net_message_at(&mut self, i: usize, id: u32, text: bool, data: &[u8]) {
        self.inner.controllers[i].net_deliver(
            id,
            NetEvent::Message {
                text,
                data: data.to_vec(),
            },
        );
    }

    /// Connection `id` of articulated `i` is gone, however it went.
    pub fn net_closed_at(&mut self, i: usize, id: u32) {
        self.inner.controllers[i].net_deliver(id, NetEvent::Closed);
    }

    pub fn reboot_code_controller(&mut self, i: usize, code: &str) {
        self.inner.controllers[i].reboot(code);
    }

    pub fn reboot_esp32_controller(&mut self, i: usize, app_bin: Vec<u8>, symbols: &str) {
        self.inner.controllers[i].reboot_esp32(app_bin, symbols);
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
            Visual::RigidMesh(_) => 2,
            Visual::Point(_) => 3,
        };
    }

    // Whether the visual in i-th articulated should be shown
    pub fn visual_show(&self, i: usize) -> bool {
        self.inner.articulated[i].show_visual
    }

    pub fn static_visual_show(&self, i: usize) -> bool {
        self.inner.static_bodies[i].show_visual
    }

    /// Return color, or default
    pub fn visual_color(&self, i: usize, j: usize, k: usize) -> Float32Array {
        let color = &self.inner.articulated[i].bodies[j].visual[k].2;
        if let Some(color) = color {
            toJsFloat32Array!(color)
        } else {
            toJsFloat32Array!([1., 0., 0.])
        }
    }

    pub fn visual_sphere_r(&self, i: usize, j: usize, k: usize) -> Float {
        let visual = &self.inner.articulated[i].bodies[j].visual[k].0;
        return match visual {
            Visual::Sphere(sphere) => sphere.r,
            _ => panic!("visual is not a sphere"),
        };
    }

    pub fn visual_cuboid_wdh(&self, i: usize, j: usize, k: usize) -> Float32Array {
        let visual = &self.inner.articulated[i].bodies[j].visual[k].0;
        return match visual {
            Visual::Cuboid(cuboid) => toJsFloat32Array!([cuboid.w, cuboid.d, cuboid.h]),
            _ => panic!("visual is not a cuboid"),
        };
    }

    pub fn visual_mesh_vertices(&self, i: usize, j: usize, k: usize) -> Float32Array {
        let visual = &self.inner.articulated[i].bodies[j].visual[k].0;
        return match visual {
            Visual::RigidMesh(mesh) => {
                let vertices = &mesh.vertices;
                let flat: Vec<Float> = vertices.iter().flat_map(|v| v.iter().cloned()).collect();
                toJsFloat32Array!(flat)
            }
            _ => panic!("visual is not a rigid mesh"),
        };
    }

    pub fn visual_mesh_faces(&self, i: usize, j: usize, k: usize) -> Uint32Array {
        let visual = &self.inner.articulated[i].bodies[j].visual[k].0;
        return match visual {
            Visual::RigidMesh(mesh) => {
                let faces = &mesh.faces;
                let flat: Vec<usize> = faces.iter().flat_map(|&f| f).collect();
                toJsUint32Array!(flat)
            }
            _ => panic!("visual is not a rigid mesh"),
        };
    }

    /// The mesh's material groups as `[start, count]` pairs, counted in *faces*.
    ///
    /// Empty for a mesh drawn in one colour -- i.e. anything but an OBJ that
    /// named materials with `usemtl` -- in which case `visual_color` is the
    /// whole story. When it is not empty the pairs cover every face, so the
    /// renderer can hand each run its own material and still draw the mesh
    /// whole. The mesh itself is undivided: one set of vertices, one collider.
    pub fn visual_mesh_groups(&self, i: usize, j: usize, k: usize) -> Uint32Array {
        let flat: Vec<usize> = self
            .mesh_draw_groups(i, j, k)
            .iter()
            .flat_map(|(start, count, _color)| [*start, *count])
            .collect();
        toJsUint32Array!(flat)
    }

    /// The colour of each group `visual_mesh_groups` returned, as RGB triples.
    pub fn visual_mesh_group_colors(&self, i: usize, j: usize, k: usize) -> Float32Array {
        let flat: Vec<Float> = self
            .mesh_draw_groups(i, j, k)
            .iter()
            .flat_map(|(_start, _count, color)| [color.x, color.y, color.z])
            .collect();
        toJsFloat32Array!(flat)
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

    pub fn deformable_nodes(&self) -> Float32Array {
        let mut q: Vec<Float> = vec![];
        for deformable in self.inner.deformables.iter() {
            q.extend(deformable.q.iter());
        }
        toJsFloat32Array!(q)
    }

    pub fn cloth_nodes(&self) -> Float32Array {
        let mut q: Vec<Float> = vec![];
        for cloth in self.inner.cloths.iter() {
            q.extend(cloth.q.iter());
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

    pub fn cloth_dofs(&self) -> Uint32Array {
        let dofs: Vec<usize> = self.inner.cloths.iter().map(|x| x.dof()).collect();
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

    pub fn cloth_faces(&self) -> Uint32Array {
        let mut faces: Vec<usize> = vec![];
        for cloth in self.inner.cloths.iter() {
            faces.extend(cloth.faces.iter().flat_map(|f| *f).collect::<Vec<usize>>());
        }
        Uint32Array::from(
            faces
                .iter()
                .map(|x| *x as u32)
                .collect::<Vec<u32>>()
                .as_slice(),
        )
    }

    /// number of faces of each deformable
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

    /// number of faces of each cloth
    pub fn cloth_face_ns(&self) -> Uint32Array {
        let face_ns: Vec<usize> = self.inner.cloths.iter().map(|x| x.faces.len()).collect();
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

    pub fn n_static_bodies(&self) -> usize {
        self.inner.static_bodies.len()
    }

    pub fn static_body_vertices(&self, i: usize) -> Float32Array {
        let mesh = &self.inner.static_bodies[i].mesh;
        let vertices = &mesh.vertices;
        let flat: Vec<Float> = vertices.iter().flat_map(|v| v.iter().cloned()).collect();
        toJsFloat32Array!(flat)
    }

    pub fn static_body_faces(&self, i: usize) -> Uint32Array {
        let mesh = &self.inner.static_bodies[i].mesh;
        let faces = &mesh.faces;
        let flat: Vec<usize> = faces.iter().flat_map(|&f| f).collect();
        toJsUint32Array!(flat)
    }

    pub fn step(&mut self, dt: Float, input: Vec<Float>) {
        let n_substep = 1;
        let dt = dt / (n_substep as Float);
        for _ in 0..n_substep {
            self.inner.step(dt, &input);
        }
    }

    pub fn step_dual(&mut self, dt: Float, input: Vec<Float>) {
        let n_substep = 1;
        let dt = dt / (n_substep as Float);
        for _ in 0..n_substep {
            self.inner.step_dual(dt, &input);
        }
    }

    pub fn set_joint_q(&mut self, i: usize, value: Float) {
        self.inner.articulated[0].set_joint_q(i, JointPosition::Float(value));
    }

    pub fn joint_q(&mut self) -> Float32Array {
        toJsFloat32Array!(self.inner.articulated[0].q())
    }

    pub fn set_joint_v(&mut self, i: usize, value: Float) {
        self.inner.articulated[0].set_joint_v(i, JointVelocity::Float(value));
    }
}

#[wasm_bindgen]
pub async fn createHybridSphereAndTetra() -> InterfaceHybrid {
    let mut state = Hybrid::empty();

    let mut sphere = Articulated::new_sphere("sphere", 1., 1.);
    sphere.set_joint_q(
        0,
        JointPosition::Pose(Pose::translation(vector![2.5, 0., 0.])),
    );
    let v_sphere = vector![-1., 0., 0.];
    sphere.set_joint_v(0, JointVelocity::Spatial(SpatialVector::linear(v_sphere)));
    state.add_articulated(sphere);

    state.add_deformable(Deformable::new_tetrahedron());

    let v = vector![0.25, 0., 0.];
    let v = vec![v, v, v, v];
    state.set_deformable_velocities(vec![v]);

    InterfaceHybrid::new(state)
}

#[wasm_bindgen]
pub async fn createHybridCube() -> InterfaceHybrid {
    let mut state = Hybrid::empty();

    // Add first sphere
    let mut sphere = Articulated::new_sphere("sphere", 1., 1.);
    sphere.set_joint_q(
        0,
        JointPosition::Pose(Pose::translation(vector![2.5, 0., 0.])),
    );
    let v_sphere = vector![-1., 0., 0.];
    sphere.set_joint_v(0, JointVelocity::Spatial(SpatialVector::linear(v_sphere)));
    state.add_articulated(sphere);

    // Add second sphere
    let mut sphere = Articulated::new_sphere("sphere", 1., 1.);
    sphere.set_joint_q(
        0,
        JointPosition::Pose(Pose::translation(vector![-1.5, 0., 0.])),
    );
    let v_sphere = vector![1., 0., 0.];
    sphere.set_joint_v(0, JointVelocity::Spatial(SpatialVector::linear(v_sphere)));
    state.add_articulated(sphere);

    state.add_deformable(Deformable::new_cube(1e2));
    // let v = vector![1. / 7., 0., 0.];
    // let v = vec![v; 7];
    // state.set_deformable_velocities(vec![v]);

    InterfaceHybrid::new(state)
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

    InterfaceHybrid::new(state)
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

    InterfaceHybrid::new(state)
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

    InterfaceHybrid::new(state)
}

#[wasm_bindgen]
pub async fn createCuboidCart() -> InterfaceHybrid {
    let mut state = Hybrid::empty();

    let l_cube = 1.0;
    let cart_offset = 0.0;

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
    state.disable_gravity();
    // let v = vector![1. / 8., 0., 0.];
    // let v = vec![v; 8];
    // state.set_deformable_velocities(vec![v]);

    InterfaceHybrid::new(state)
}

#[wasm_bindgen]
pub async fn createGripperHybrid() -> InterfaceHybrid {
    let state = build_gripper_cube();

    InterfaceHybrid::new(state)
}

#[wasm_bindgen]
pub async fn createTeddy() -> InterfaceHybrid {
    let buf = read_web_file("teddy.vtk").await;
    let state = build_teddy(&buf);

    InterfaceHybrid::new(state)
}

#[wasm_bindgen]
pub async fn createCubeFrenzy() -> InterfaceHybrid {
    let state = build_cube_frenzy();

    InterfaceHybrid::new(state)
}
