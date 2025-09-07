use na::{vector, UnitQuaternion, Vector3};

use crate::{fem::cloth::Cloth, types::Float};

/// Build a cloth of m nodes in x-axis, and n nodes in y-axis, of spacing dx
pub fn build_cloth(m: usize, n: usize, dx: Float, rotation: UnitQuaternion<Float>) -> Cloth {
    let mut vertices = vec![];
    for i in 0..n {
        for j in 0..m {
            vertices.push(rotation * vector![j as Float * dx, i as Float * dx, 0.]);
        }
    }

    let mut triangles = vec![];
    for i in 0..n - 1 {
        for j in 0..m - 1 {
            let v0 = i * m + j;
            let v1 = v0 + 1;
            let v2 = v0 + m;
            let v3 = v2 + 1;

            triangles.push([v0, v1, v2]);
            triangles.push([v1, v3, v2]);
        }
    }

    Cloth::new(vertices, triangles)
}
