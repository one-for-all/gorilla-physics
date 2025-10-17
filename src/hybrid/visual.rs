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
