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
    fn eeprom_float(&mut self, addr: usize) -> Float {
        0.
    }

    fn step(&mut self, dt: Float, articulated: &Articulated) {}

    fn control(&mut self, articulated: &Articulated, input: &Vec<Float>) -> DVector<Float>;
}

pub struct NullArticulatedController {}

impl ArticulatedController for NullArticulatedController {
    fn control(&mut self, articulated: &Articulated, input: &Vec<Float>) -> DVector<Float> {
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

        let base_mass = 10.;
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
