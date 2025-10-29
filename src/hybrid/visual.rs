use na::{vector, Isometry3, Point3, UnitVector3, Vector3};

use crate::types::Float;

pub struct SphereGeometry {
    pub r: Float,
}

pub struct CuboidGeometry {
    pub w: Float, // width in x-axis
    pub d: Float, // depth in y-axis
    pub h: Float, // height in z-axis
}

impl CuboidGeometry {
    /// Returns all the corner points of the cuboid
    /// Ordered as left-front-bottom, left-front-top, left-back-bottom, left-back-top, right-[]
    pub fn points(&self, iso: &Isometry3<Float>) -> Vec<Vector3<Float>> {
        let mut points = vec![];
        let multipliers = [-0.5, 0.5];
        for iw in multipliers {
            for id in multipliers {
                for ih in multipliers {
                    points.push(vector![iw * self.w, id * self.d, ih * self.h]);
                }
            }
        }

        points
            .iter()
            .map(|p| iso.transform_point(&Point3::from(*p)).coords)
            .collect()
    }

    /// Returns all the edges of the cuboid
    pub fn edges(&self, iso: &Isometry3<Float>) -> Vec<[Vector3<Float>; 2]> {
        let points = self.points(iso);
        let edges = vec![
            // left face
            [0, 1],
            [1, 3],
            [3, 2],
            [2, 0],
            // front face
            [0, 4],
            [4, 5],
            [5, 1],
            // top face
            [5, 7],
            [7, 3],
            // back face
            [7, 6],
            [6, 2],
            // right face
            [6, 4],
        ];
        assert_eq!(edges.len(), 12);
        edges.iter().map(|e| [points[e[0]], points[e[1]]]).collect()
    }

    /// Returns all the faces of the cuboid, represented by (vertex point, edge vector 1, edge vector 2), where edge vector is the vector from that vertex to a neighbor vertex, and edge 1 cross edge 2 gives the outward normal direction
    pub fn faces(&self, iso: &Isometry3<Float>) -> Vec<[Vector3<Float>; 3]> {
        let points = self.points(iso);
        let faces = vec![
            [0, 1, 2], // left face
            [0, 4, 1], // front face
            [1, 5, 3], // top face
            [3, 7, 2], // back face
            [0, 2, 4], // bottom face
            [4, 6, 5], // right face
        ];
        faces
            .iter()
            .map(|f| {
                let v1 = points[f[0]];
                let v2 = points[f[1]];
                let v3 = points[f[2]];
                [v1, v2 - v1, v3 - v1]
            })
            .collect()
    }
}

/// Returns (contact point, contact normal) if vertex collides with rectangular face.
/// normal point outwards from the face
pub fn vertex_rect_face_collision(
    vertex: &Vector3<Float>,
    face: &[Vector3<Float>; 3],
    tol: Float,
) -> Option<(Vector3<Float>, UnitVector3<Float>)> {
    let v = face[0];
    let e1 = face[1];
    let e2 = face[2];
    let n = UnitVector3::new_normalize(e1.cross(&e2));

    let distance = (vertex - v).dot(&n);
    if distance.abs() > tol {
        return None;
    }

    // point projected from vertex to face, subtracted from v, to give the vector pointing from v to the projection.
    let project = vertex - n.scale(distance) - v;

    let proj_e1 = project.dot(&e1) / e1.norm_squared();
    if proj_e1 > 1.0 || proj_e1 < 0.0 {
        return None;
    }

    let proj_e2 = project.dot(&e2) / e2.norm_squared();
    if proj_e2 > 1.0 || proj_e2 < 0.0 {
        return None;
    }

    Some((project, n))
}

pub enum Visual {
    Sphere(SphereGeometry),
    Cuboid(CuboidGeometry),
}

impl Visual {
    pub fn new_cuboid(w: Float, d: Float, h: Float) -> Visual {
        Visual::Cuboid(CuboidGeometry { w, d, h })
    }

    pub fn cuboid(&self) -> &CuboidGeometry {
        match self {
            Self::Cuboid(cuboid) => cuboid,
            _ => panic!("not cuboid"),
        }
    }
}
