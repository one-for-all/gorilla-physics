use na::{vector, Vector3};

use crate::types::Float;

use super::cuboid::Cuboid;

/// Minkowski polytope vertex
#[derive(Clone)]
pub struct PolytopeVertex {
    pub v: Vector3<Float>,
    pub index_A: usize, // index of A that gives this vertex
    pub index_B: usize, // index of B that gives this vertex
}

/// Minkowski polytope
pub struct Polytope {
    pub vertices: Vec<PolytopeVertex>,       // vertices of the polytope
    pub triangles: Vec<Vector3<usize>>, // vertex indices stored clockwise order, when viewed from inside. i.e. v1v2 cross v1v3 points outward
    pub closest_points: Vec<Vector3<Float>>, // closest point on triangle face to origin
    pub obsolete: Vec<bool>, // keeps track of whether the triangle is still part of the polytope
}

impl Polytope {
    /// Construct tetrahedron from (v1, v2, v3, v4) where (v1, v2, v3) forms the
    /// base triangle in clockwise order viewed from inside
    pub fn from_simplex(vertices: &Vec<PolytopeVertex>) -> Self {
        Polytope {
            vertices: vertices.clone(),
            triangles: vec![
                vector![0, 1, 2],
                vector![0, 3, 1],
                vector![0, 2, 3],
                vector![1, 3, 2],
            ],
            closest_points: vec![],
            obsolete: vec![false, false, false, false],
        }
    }

    pub fn contains_origin(&self) -> bool {
        for triangle in &self.triangles {
            let v1 = &self.vertices[triangle[0]];
            let v2 = &self.vertices[triangle[1]];
            let v3 = &self.vertices[triangle[2]];

            let v1v2 = v2.v - v1.v;
            let v1v3 = v3.v - v1.v;
            let normal = v1v2.cross(&v1v3);
            if normal.dot(&v1.v) < 0.0 {
                println!("violating tri: {}, amount: {}", triangle, normal.dot(&v1.v));
                return false;
            }
        }

        true
    }
}

pub fn support_fn(A: &Cuboid, B: &Cuboid, direction: &Vector3<Float>) -> PolytopeVertex {
    let (sp_a, index_A) = A.compute_support(direction);
    let (sp_b, index_B) = B.compute_support(&-direction);

    PolytopeVertex {
        v: sp_a - sp_b,
        index_A,
        index_B,
    }
}
