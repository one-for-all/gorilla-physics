use na::{vector, Isometry, Isometry3, Point3, Vector3};

use crate::{collision::cuboid, types::Float};

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
