use na::{dvector, DVector};

use crate::{
    control::navbot_control::PIModule, flog, hybrid::articulated::Articulated, types::Float,
    GRAVITY,
};

pub trait ArticulatedController {
    fn control(&mut self, articulated: &Articulated, input: &Vec<Float>) -> DVector<Float>;
}

pub struct NullArticulatedController {}

impl ArticulatedController for NullArticulatedController {
    fn control(&mut self, articulated: &Articulated, input: &Vec<Float>) -> DVector<Float> {
        DVector::zeros(articulated.dof())
    }
}

pub struct GripperController {
    pi_vz: PIModule,
    pi_qgrip: PIModule,
}

impl GripperController {
    pub fn new(dt: Float) -> Self {
        Self {
            pi_vz: PIModule::new(200., 200., 1000., dt),
            pi_qgrip: PIModule::new(200., 0., 300., dt),
        }
    }
}

impl ArticulatedController for GripperController {
    fn control(&mut self, articulated: &Articulated, input: &Vec<Float>) -> DVector<Float> {
        let vz = articulated.v()[0];
        let vz_target = if input[1] > 0. {
            1.0
        } else if input[1] < 0. {
            -1.0
        } else {
            0.
        };
        let mut fz = self.pi_vz.compute(vz_target - vz);
        flog!("fz: {}", fz);

        fz += articulated.mass() * GRAVITY;

        let q_grip = articulated.q()[1];
        let q_grip_target = if input[2] > 0. { 1.0 } else { 0. };
        let f_grip = self.pi_qgrip.compute(q_grip_target - q_grip);
        flog!("fgrip: {}", f_grip);
        dvector![fz, f_grip, -f_grip]
    }
}
