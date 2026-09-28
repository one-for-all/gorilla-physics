use na::{dvector, DVector};

use crate::{
    control::navbot_control::PIModule, hybrid::articulated::Articulated, types::Float, GRAVITY,
};

pub trait ArticulatedController {
    /// Access the float stored in DDRB register
    /// TODO: make this cleaner
    fn ddrb_float(&mut self) -> Float {
        0.
    }

    /// Access the float stored in eeprom addr
    /// TODO: make this cleaner
    fn eeprom_float(&mut self, _addr: usize) -> Float {
        0.
    }

    fn step(&mut self, _dt: Float, _articulated: &Articulated) {}

    fn control(&mut self, articulated: &Articulated, input: &Vec<Float>) -> DVector<Float>;

    /// re-boot the microcontroller to run code
    fn reboot(&mut self, _code: &str) {}

    fn reboot_esp32(&mut self, _app_bin: Vec<u8>, _symbols: &str) {}

    /// Return data for debugging
    fn debug_data(&self) -> Float {
        0.
    }

    fn debug(&mut self) {}

    fn get_uart(&self) -> String {
        "".to_string()
    }

    fn send_uart(&mut self, _payload: &str) {}

    /// Press or release a switch wired to this controller's board, as a finger
    /// on the real part would. Switches are numbered in the order the hardware
    /// was assembled, which is the order the thing that built the controller
    /// listed them in.
    fn set_switch(&mut self, _switch: usize, _pressed: bool) {}

    /// Hold joystick `joystick` at `x`, `y` -- each in [-1, 1], with 0
    /// released -- and press or release the switch under its cap.
    ///
    /// One call rather than three, because that is what the host has: a
    /// pointer on a pad knows where the stick is and whether the button is
    /// down, all at once, and nothing downstream cares which of the three
    /// changed. Joysticks are numbered as switches are, in the order the
    /// thing that built the controller listed them.
    fn set_joystick(&mut self, _joystick: usize, _x: Float, _y: Float, _pressed: bool) {}

    /// What is wrong with the hardware as it stands, if anything -- a wiring
    /// fault that only the running firmware can reveal, such as a switch tied
    /// to a pin the sketch has made an output. One message rather than a list,
    /// for the same reason the wiring panel shows one: the next fault surfaces
    /// as soon as this one is fixed.
    fn hardware_warning(&self) -> Option<String> {
        None
    }

    /// The frame the emulated camera `camera` wants from the host right now,
    /// or `None` while there is no camera there or its firmware has not
    /// configured one. Cameras are numbered as the thing that built the
    /// controller listed them, exactly as switches are.
    ///
    /// The request changes as the sketch runs -- `esp_camera_init` is what
    /// programs a sensor -- so the host polls this rather than being told
    /// once.
    fn camera_request(&self, _camera: usize) -> Option<CameraFrameRequest> {
        None
    }

    /// Hand camera `camera` its next picture, as tightly packed RGBA bytes
    /// (`width * height * 4`, top row first) at the size the current
    /// [`camera_request`](Self::camera_request) asked for. The controller owns
    /// turning pixels into whatever its sensor puts on the wire.
    fn supply_camera_frame(&mut self, _camera: usize, _rgba: &[u8]) {}

    /// What the board's network wants of the host right now, taken away as
    /// it is read.
    ///
    /// Polled like [`camera_request`](Self::camera_request), and for the same
    /// reason: what a sketch does with a socket is the running firmware's
    /// business, and there is no moment at which the host could be told once.
    /// A board without a radio never asks for anything.
    fn net_take_requests(&mut self) -> Vec<NetRequest> {
        Vec::new()
    }

    /// Tell the board what became of connection `id`.
    fn net_deliver(&mut self, _id: u32, _event: NetEvent) {}
}

/// What a board's firmware wants of the network.
///
/// The engine carries these without acting on them: it neither opens
/// connections nor knows what one means. It is a conduit for the same reason
/// [`CameraFrameRequest`] is -- the controller is a `Box<dyn>` inside a
/// [`crate::hybrid::Hybrid`], so nothing downstream can reach the hardware it
/// built.
#[derive(Debug, Clone, PartialEq)]
pub enum NetRequest {
    /// Open a WebSocket to `ws://host:port/path`.
    Open {
        id: u32,
        host: String,
        port: u16,
        path: String,
    },
    /// Send a message on connection `id`.
    Send { id: u32, text: bool, data: Vec<u8> },
    /// Close connection `id`.
    Close { id: u32 },
}

/// What the host has to say back about a connection.
#[derive(Debug, Clone, PartialEq)]
pub enum NetEvent {
    /// The connection is up, and not before: firmware told otherwise writes
    /// into a hole.
    Opened,
    /// A message arrived.
    Message { text: bool, data: Vec<u8> },
    /// The connection is gone -- closed, refused, or never established.
    Closed,
}

/// What an emulated camera is configured to produce -- everything a host
/// needs to render and pack one frame for it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CameraFrameRequest {
    pub format: CameraFrameFormat,
    pub width: u32,
    pub height: u32,
    /// Frames per second the sensor is streaming at; supplying frames faster
    /// than this is wasted work, since the stream picks up one per cycle.
    pub fps: Float,
    /// Whether the sensor reverses the order of its scan lines before they
    /// reach the board. The host must apply this before supplying pixels.
    pub vertical_flip: bool,
}

/// How the frame's pixels go on the wire.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CameraFrameFormat {
    Jpeg,
    Rgb565,
    Grayscale,
}

pub struct NullArticulatedController {}

impl ArticulatedController for NullArticulatedController {
    fn control(&mut self, articulated: &Articulated, _input: &Vec<Float>) -> DVector<Float> {
        DVector::zeros(articulated.dof())
    }
}

pub struct GripperController {
    pi_vx: PIModule,
    pi_vz: PIModule,
    pi_qgrip: PIModule,
    pi_vgrip: PIModule,
}

impl GripperController {
    pub fn new(dt: Float) -> Self {
        Self {
            pi_vx: PIModule::new(200., 200., 1000., dt),
            pi_vz: PIModule::new(200., 200., 1000., dt),
            pi_qgrip: PIModule::new(400., 10., 500., dt),
            pi_vgrip: PIModule::new(100., 0., 1000., dt),
        }
    }
}

impl ArticulatedController for GripperController {
    fn control(&mut self, articulated: &Articulated, input: &Vec<Float>) -> DVector<Float> {
        let v = articulated.v();
        let q = articulated.q();

        let x_joint = 1;
        let vx = v[x_joint];
        let vx_target = input[0];
        let fx = self.pi_vx.compute(vx_target - vx);

        let z_joint = 0;
        let vz = v[z_joint];
        let vz_target = if input[1] > 0. {
            1.0
        } else if input[1] < 0. {
            -1.0
        } else {
            0.
        };
        let mut fz = self.pi_vz.compute(vz_target - vz);

        let _base_mass = 10.;
        fz += (articulated.mass()) * GRAVITY;

        let q_closed = 1.5 / 2.;
        // left finger
        let right_joint = 2;
        let q_left = q[right_joint];
        let q_left_target = if input[2] > 0. { q_closed } else { 0. };
        let f_qleft = self.pi_qgrip.compute(q_left_target - q_left);

        let v_left = v[right_joint];
        let v_left_target = 0.;
        let f_vleft = self.pi_vgrip.compute(v_left_target - v_left);

        let f_left = f_qleft + f_vleft;

        // right right
        let right_joint = 3;
        let q_right = q[right_joint];
        let q_right_target = if input[2] > 0. { -q_closed } else { 0. };
        let f_qright = self.pi_qgrip.compute(q_right_target - q_right);

        let v_right = v[right_joint];
        let v_right_target = 0.;
        let f_vright = self.pi_vgrip.compute(v_right_target - v_right);

        let f_right = f_qright + f_vright;

        dvector![fz, fx, f_left, f_right]
    }
}
