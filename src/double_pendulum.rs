use na::{Matrix2, Vector2};

use crate::{types::Float, GRAVITY};

/// Manual writing out of the manipulator equation for simple double pendulum
/// Ref: https://underactuated.csail.mit.edu/multibody.html#manipulator
pub struct SimpleDoublePendulum {
    M: Matrix2<Float>,
    dynamics_bias: Vector2<Float>,
}

impl SimpleDoublePendulum {
    #[rustfmt::skip]
    pub fn new(m1: Float, m2: Float, l1: Float, l2: Float, q1: Float, q2: Float, q1dot: Float, q2dot: Float) -> Self {
        let s1 = q1.sin();
        let s2 = q2.sin();
        let s12 = (q1+q2).sin();
        let c2 = q2.cos();
        let I2 = m2*l2*l2;
        let m12 = I2 + m2*l1*l2*c2;
        let M = Matrix2::new(
            (m1 + m2)*l1*l1 + I2 + 2.*m2*l1*l2*c2, m12,
            m12                                  ,  I2
        );
        let C = Matrix2::new(
            0.0,                                -m2*l1*l2*(2.*q1dot + q2dot)*s2,
            0.5*m2*l1*l2*(2.*q1dot + q2dot)*s2, -0.5*m2*l1*l2*q1dot*s2
        );
        let tau_g = -GRAVITY*Vector2::new(
            (m1+m2)*l1*s1 + m2*l2*s12,
            m2*l2*s12
        );

        let dynamics_bias = C * Vector2::new(q1dot, q2dot) - tau_g;

        SimpleDoublePendulum {
            M, dynamics_bias
        }
    }

    pub fn dynamics(&self) -> Vector2<Float> {
        if let Some(vdot) = self.M.clone().lu().solve(&(-self.dynamics_bias)) {
            vdot
        } else {
            panic!(
                r#"Failed to solve for vdot in M(q) vdot + c(q, v) = 0
            where M = {}, 
                  c = {}
            "#,
                self.M, self.dynamics_bias
            )
        }
    }
}

#[cfg(test)]
mod double_pendulum_tests {

    use na::{dvector, DVector};

    use crate::util::assert_close;

    use super::*;

    #[test]
    fn dynamics() {
        // Arrange
        let m1 = 1.0;
        let l1 = 2.0;
        let m2 = 3.0;
        let l2 = 4.0;

        let q1 = 1.0;
        let q2 = 2.0;
        let q1dot = 3.0;
        let q2dot = 4.0;

        let dp = SimpleDoublePendulum::new(m1, m2, l1, l2, q1, q2, q1dot, q2dot);

        // Act
        let vdot = DVector::from_column_slice(dp.dynamics().as_slice());

        // Assert
        assert_close(&vdot, &dvector![68.8824, -58.9877], 1e-4); // reference values from RigidBodyDynamics.jl
    }
}
