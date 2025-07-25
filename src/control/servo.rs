use crate::types::Float;

pub struct ServoMotor {
    P: Float,
    I: Float,
    pub D: Float,
    dt: Float, // motor control frequency

    target: Float,

    pos: Float,
    vel: Float,
    integral: Float,
}

impl ServoMotor {
    pub fn new(P: Float, I: Float, D: Float, dt: Float) -> Self {
        Self {
            P,
            I,
            D,
            dt,
            target: 0.,

            pos: 0.,
            vel: 0.,
            integral: 0.,
        }
    }

    pub fn update(&mut self, pos: Float, vel: Float) {
        self.pos = pos;
        self.vel = vel;
        self.integral += (self.target - self.pos) * self.dt;
    }

    pub fn set(&mut self, target: Float) {
        self.target = target
    }

    pub fn compute(&mut self) -> Float {
        let ctrl = self.target;
        let error = ctrl - self.pos;
        let error_dot = error - self.vel;
        // let error_dot = 0. - self.vel;

        // let integral = self.integral + error * self.dt;

        self.P * error + self.D * error_dot + self.I * self.integral
    }
}
