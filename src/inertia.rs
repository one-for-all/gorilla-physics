use std::ops::{Add, AddAssign};

use na::{DMatrix, Point3, SMatrix, Vector3};
use nalgebra::{vector, Matrix3};

use crate::{spatial::transform::Transform3D, spatial::twist::Twist, types::Float};
use itertools::izip;
use std::collections::HashMap;

use crate::mechanism::MechanismState;

/// A spatial inertia, or inertia matrix, represents the mass distribution of a
/// rigid body.
/// A spatial inertia expressed in frame i is defined as:
/// I^i = | J         c_hat |
///       | c_hat^T     mI  |
/// where J is the mass moment of inertia, m is the total mass, and c is the
/// 'cross part', which is the center of mass position scaled by m.
/// Written as an integration:
/// I^i = ∫ over B, ρ(x) * | p_hat^T(x)p_hat(x)     p_hat(x) |
///                        | p_hat^T(x)             I        | dx
///
/// !!! Warning
///     The __moment__ field of a __SpatialInertia__ is the moment of inertia
///     about the origin of its __frame__, not about the center of mass.
#[derive(Debug, Clone, PartialEq)]
pub struct SpatialInertia {
    pub frame: String,
    pub moment: Matrix3<Float>,
    pub cross_part: Vector3<Float>,
    pub mass: Float,
}

impl SpatialInertia {
    // TODO: Auto-check positive-definiteness
    pub fn new(
        moment: Matrix3<Float>,
        cross_part: Vector3<Float>,
        mass: Float,
        frame: &str,
    ) -> Self {
        SpatialInertia {
            frame: frame.to_string(),
            moment,
            cross_part,
            mass,
        }
    }

    pub fn center_of_mass(&self) -> Point3<Float> {
        let p = self.cross_part / self.mass;
        Point3::<Float>::new(p.x, p.y, p.z)
    }

    pub fn default() -> Self {
        SpatialInertia {
            frame: "world".to_string(),
            moment: SMatrix::identity(),
            cross_part: vector![1.0, 1.0, 1.0],
            mass: 1.0,
        }
    }

    // Tranform the spatial inertia to be expressed in world frame
    pub fn transform(&self, transform: &Transform3D) -> SpatialInertia {
        if self.frame != transform.from {
            panic!(
                "self frame {} and transform from frame {} do not match!",
                self.frame, transform.from
            );
        }

        let R = transform.rot();
        let p = transform.trans();

        let J = self.moment;
        let mc = self.cross_part;
        let m = self.mass;

        let Rmc = R * mc;
        let mp = m * p;
        let mcnew = Rmc + mp;
        let X = Rmc * p.transpose();
        let Y = X + X.transpose() + mp * p.transpose();
        let Jnew = R * J * R.transpose() - Y + Y.trace() * DMatrix::identity(Y.nrows(), Y.ncols());

        SpatialInertia {
            frame: "world".to_string(),
            moment: Jnew,
            cross_part: mcnew,
            mass: m,
        }
    }
}

impl<'a, 'b> Add<&'b SpatialInertia> for &'a SpatialInertia {
    type Output = SpatialInertia;

    /// lhs is A to B twist, rhs is B to C twist,
    /// returns A to C twist.
    fn add(self, rhs: &SpatialInertia) -> SpatialInertia {
        SpatialInertia {
            frame: self.frame.clone(),
            moment: self.moment + rhs.moment,
            cross_part: self.cross_part + rhs.cross_part,
            mass: self.mass + rhs.mass,
        }
    }
}

impl<'a, 'b> AddAssign<&'b SpatialInertia> for SpatialInertia {
    fn add_assign(&mut self, rhs: &Self) {
        if self.frame != rhs.frame {
            panic!("lhs frame {} != rhs frame {}!", self.frame, rhs.frame);
        }

        self.moment += rhs.moment;
        self.cross_part += rhs.cross_part;
        self.mass += rhs.mass;
    }
}

/// Compute the inertia of each body expressed in world frame.
pub fn compute_inertias(
    state: &MechanismState,
    bodies_to_root: &Vec<Transform3D>,
) -> HashMap<usize, SpatialInertia> {
    let mut inertias = HashMap::new();
    for (jointid, body) in izip!(state.treejointids.iter(), state.bodies.iter()) {
        let bodyid = jointid;
        let body_to_root = &bodies_to_root[*bodyid];
        let inertia = body.inertia.transform(body_to_root);
        inertias.insert(*bodyid, inertia);
    }

    inertias
}

/// Computes the kinetic energy of a body
/// Essentially implements KE = 1/2 * v^T * M * v
pub fn kinetic_energy(inertia: &SpatialInertia, twist: &Twist) -> Float {
    // Ensure both are expressed in world frame
    if inertia.frame != "world" {
        panic!("spatial inertia frame {} is not world.", inertia.frame);
    }
    if twist.frame != "world" {
        panic!("twist frame {} is not world.", twist.frame);
    }

    if twist.base != "world" {
        panic!("twist base {} is not world.", twist.base);
    }

    let w = twist.angular;
    let v = twist.linear;
    let J = inertia.moment;
    let c = inertia.cross_part;
    let m = inertia.mass;

    (w.dot(&(J * w)) + v.dot(&(m * v + 2.0 * w.cross(&c)))) / 2.0
}
