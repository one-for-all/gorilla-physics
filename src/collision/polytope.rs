use na::{vector, Vector3};

use crate::types::Float;

/// Vertex of a Minkowski polytope of (A-B)
#[derive(Debug)]
pub struct Vertex {
    pub v: Vector3<Float>,    // vertex coordinates
    pub sp_a: Vector3<Float>, // support point from shape A
    pub sp_b: Vector3<Float>, // support point from shape B
}

impl Vertex {
    pub fn new(sp_a: Vector3<Float>, sp_b: Vector3<Float>) -> Vertex {
        Vertex {
            v: sp_a - sp_b, // vertex coordinates computed from support points
            sp_a,
            sp_b,
        }
    }
}

/// Minkowski polytope
#[derive(Debug)]
pub struct Polytope {
    pub vertices: Vec<Vertex>,
    pub triangles: Vec<Vector3<usize>>, // vertex indices stored counter-clockwise order, when viewed from outside
}

impl Polytope {
    pub fn empty() -> Polytope {
        Polytope {
            vertices: vec![],
            triangles: vec![],
        }
    }

    /// Construct tetrahedron from (v0, v1, v2, v3) where (v0, v1, v2) forms the
    /// base triangle in clockwise order viewed from outside
    pub fn tetrahedron(vertices: Vec<Vertex>) -> Polytope {
        Polytope {
            vertices,
            triangles: vec![
                vector![0, 2, 1],
                vector![0, 1, 3],
                vector![0, 3, 2],
                vector![1, 2, 3],
            ],
        }
    }
}
