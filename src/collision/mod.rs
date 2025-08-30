use cuboid::Cuboid;
use epa::epa;
use gjk::gjk;
use na::{Translation3, Vector3};
use polytope::Polytope;

use crate::types::Float;

pub mod ccd;
pub mod cuboid;
pub mod epa;
pub mod gjk;
pub mod halfspace;
pub mod mesh;
pub mod polytope;
pub mod sphere;
pub mod triangle;

pub fn addEdgeIfNotExisting(E: &mut Vec<(usize, usize)>, edge: (usize, usize)) {
    // remove reverse edge
    let index = E.iter().position(|x| *x == (edge.1, edge.0));
    if let Some(index) = index {
        E.remove(index);
    } else {
        E.push(edge);
    }

    // procedure to perform if E is HashSet
    // if !E.remove(&(edge.1, edge.0)) {
    //     // add if not existing
    //     E.insert(edge);
    // }
}

/// For detecting collision and computing contact points between two colliders
pub struct CollisionDetector {
    // Translated reference frame at which collision detection algorithms will be
    // performed.
    // Use this frame rather than world frame for numerical
    // robustness when colliders are far from origin.
    pub translation: Translation3<Float>,
    pub A: Cuboid,
    pub B: Cuboid,

    pub poly: Option<Polytope>, // Result from GJK
}

impl CollisionDetector {
    pub fn new(A: &Cuboid, B: &Cuboid) -> Self {
        let translation = A.isometry.translation; // Take world frame translated to A as reference frame

        let mut A_clone = A.clone();
        A_clone
            .isometry
            .append_translation_mut(&translation.inverse());
        A_clone.recompute_points();
        let mut B_clone = B.clone();
        B_clone
            .isometry
            .append_translation_mut(&translation.inverse());
        B_clone.recompute_points();

        CollisionDetector {
            translation,
            A: A_clone,
            B: B_clone,
            poly: None,
        }
    }

    pub fn gjk(&mut self) -> bool {
        self.poly = gjk(&self.A, &self.B);
        return self.poly.is_some();
    }

    /// Run EPA algorithm to compute contact points.
    /// Should be run only after .gjk() gives a collision confirmation
    pub fn epa(&mut self) -> (Vector3<Float>, Vector3<Float>) {
        let poly = self.poly.as_mut().unwrap();
        let (cp_A, cp_B) = epa(poly, &self.A, &self.B);
        (
            cp_A + self.translation.vector,
            cp_B + self.translation.vector,
        )
    }
}
