//! A cuboid placed in the world, and the queries the collision routines put to it.

use na::{Isometry3, Point3, UnitVector3, Vector3};

use crate::{
    hybrid::{
        collision::shared::{Aabb, Contact},
        visual::CuboidGeometry,
    },
    types::Float,
};

/// A cuboid placed in the world, carrying the quantities every pass over it
/// needs: its half extents, and its own three axes in world frame
pub struct OrientedBox<'a> {
    /// Where it sits
    pub iso: &'a Isometry3<Float>,
    /// How big it is
    pub geometry: &'a CuboidGeometry,
    /// How far it reaches from its center along each of its own axes
    pub half: Vector3<Float>,
    /// Its own axes, in world frame
    pub axes: [Vector3<Float>; 3],
}

impl<'a> OrientedBox<'a> {
    pub fn new(iso: &'a Isometry3<Float>, geometry: &'a CuboidGeometry) -> Self {
        let rotation = iso.rotation.to_rotation_matrix();
        let axes = core::array::from_fn(|i| rotation.matrix().column(i).into_owned());

        Self {
            iso,
            geometry,
            half: geometry.half_extents(),
            axes,
        }
    }

    pub fn center(&self) -> Vector3<Float> {
        self.iso.translation.vector
    }

    /// Its eight corners, in world frame
    pub fn corners(&self) -> Vec<Vector3<Float>> {
        self.geometry.points(self.iso)
    }

    /// Its twelve edges, in world frame
    pub fn edges(&self) -> Vec<[Vector3<Float>; 2]> {
        self.geometry.edges(self.iso)
    }

    /// Its world-frame bounds. A box of half-extent `half` reaches |R| * half
    /// along each world axis.
    pub fn aabb(&self) -> Aabb {
        let extent = self.iso.rotation.to_rotation_matrix().matrix().abs() * self.half;

        Aabb::centered(&self.center(), &extent)
    }

    /// How far it reaches from its center along `direction`, a unit vector
    pub fn radius(&self, direction: &Vector3<Float>) -> Float {
        (0..3)
            .map(|i| self.half[i] * self.axes[i].dot(direction).abs())
            .sum()
    }

    /// The point of the box closest to `point`, which is `point` itself when it
    /// is inside
    ///
    /// Accumulated in world frame -- project onto each of the box's axes, clamp
    /// to the half extent, and step that far along the axis -- rather than by
    /// round-tripping `point` through the box's own frame, which keeps the
    /// result numerically stable from frame to frame.
    pub fn closest_point(&self, point: &Vector3<Float>) -> Vector3<Float> {
        let center = self.center();
        let d = point - center;

        (0..3).fold(center, |closest, axis| {
            let along = d
                .dot(&self.axes[axis])
                .clamp(-self.half[axis], self.half[axis]);
            closest + along * self.axes[axis]
        })
    }

    /// The contact of a point inside the box -- or within `tol` of its surface
    /// -- against it, pushed back out through the face it is nearest to, i.e.
    /// the face it takes the least movement to leave through
    ///
    /// The normal points out of the box.
    pub fn point_contact(&self, point: &Vector3<Float>, tol: Float) -> Option<Contact> {
        // Transform point into the box's own frame
        let local_point = self
            .iso
            .inverse_transform_point(&Point3::from(*point))
            .coords;

        // Early-out AABB check
        for axis in 0..3 {
            if local_point[axis].abs() > self.half[axis] + tol {
                return None;
            }
        }

        // The point is inside the box; find the face of least penetration
        let mut min_depth = Float::MAX;
        let mut local_normal = Vector3::zeros();
        for axis in 0..3 {
            let depth = self.half[axis] - local_point[axis].abs();
            if depth < min_depth {
                min_depth = depth;
                let mut n = Vector3::zeros();
                n[axis] = if local_point[axis] >= 0. { 1. } else { -1. };
                local_normal = n;
            }
        }

        let normal_world = self.iso.transform_vector(&local_normal);

        Some(Contact::new(
            *point,
            UnitVector3::new_normalize(normal_world),
        ))
    }

    /// The face pointing the furthest along `direction`
    pub fn support_face(&self, direction: &Vector3<Float>) -> BoxFace {
        let mut i_axis = 0;
        for i in 1..3 {
            if self.axes[i].dot(direction).abs() > self.axes[i_axis].dot(direction).abs() {
                i_axis = i;
            }
        }

        let sign = if self.axes[i_axis].dot(direction) >= 0. {
            1.
        } else {
            -1.
        };
        let normal = sign * self.axes[i_axis];
        let center = self.center() + self.half[i_axis] * normal;
        let (j, k) = ((i_axis + 1) % 3, (i_axis + 2) % 3);

        BoxFace {
            center,
            normal,
            sides: [self.half[j] * self.axes[j], self.half[k] * self.axes[k]],
        }
    }

    /// The edge along its axis `i_axis` that reaches furthest along `direction`
    pub fn support_edge(&self, i_axis: usize, direction: &Vector3<Float>) -> [Vector3<Float>; 2] {
        let mut center = self.center();
        for i in 0..3 {
            if i == i_axis {
                continue;
            }
            let sign = if self.axes[i].dot(direction) >= 0. {
                1.
            } else {
                -1.
            };
            center += sign * self.half[i] * self.axes[i];
        }

        let offset = self.half[i_axis] * self.axes[i_axis];

        [center - offset, center + offset]
    }
}

/// A face of an [`OrientedBox`], as its center, its outward normal, and its two
/// side axes each scaled to the half extent along it
pub struct BoxFace {
    pub center: Vector3<Float>,
    pub normal: Vector3<Float>,
    pub sides: [Vector3<Float>; 2],
}

impl BoxFace {
    /// Its four corners, in winding order
    pub fn corners(&self) -> [Vector3<Float>; 4] {
        [
            self.center + self.sides[0] + self.sides[1],
            self.center + self.sides[0] - self.sides[1],
            self.center - self.sides[0] - self.sides[1],
            self.center - self.sides[0] + self.sides[1],
        ]
    }
}
