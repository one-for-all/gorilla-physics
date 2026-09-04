//! Contacts between a cuboid and the simpler colliders: a sphere, and a point.

use na::{Isometry3, Point3, UnitVector3, Vector3};

use crate::{
    hybrid::{
        collision::{oriented_box::OrientedBox, shared::Contact},
        visual::{CuboidGeometry, SphereGeometry},
    },
    types::Float,
};

/// Collision detection between a sphere and a cuboid
/// Returns the contact point, and the contact normal from the sphere to the
/// cuboid, if the two are in contact
///
/// The closest point on the cuboid is built in world frame, by projecting onto
/// the cuboid's world-frame axes, rather than by round-tripping the sphere
/// center through the cuboid-local frame; that keeps the contact point
/// numerically stable from frame to frame. `sphere_cuboid_collide_local` is the
/// other way round, and the two agree.
pub fn sphere_cuboid_collide(
    sphere_center: &Vector3<Float>,
    sphere_geometry: &SphereGeometry,
    cuboid_iso: &Isometry3<Float>,
    cuboid_geometry: &CuboidGeometry,
) -> Vec<Contact> {
    let cuboid = OrientedBox::new(cuboid_iso, cuboid_geometry);
    let r = sphere_geometry.r;

    let closest = cuboid.closest_point(sphere_center);
    let diff = closest - sphere_center;
    if diff.norm_squared() > r * r {
        return vec![];
    }

    // The normal runs from the sphere center towards the contact point
    vec![Contact::new(closest, UnitVector3::new_normalize(diff))]
}

/// Collision detection between a sphere and a cuboid, worked in the cuboid's own
/// frame
///
/// Kept alongside [`sphere_cuboid_collide`] as the straightforward statement of
/// what that one computes: bring the sphere center into the cuboid's frame,
/// where the cuboid is an AABB and the closest point is a clamp, and take the
/// result back out again. The two agree to rounding; the world-frame one is what
/// the simulation runs, because it does not lose precision on the round trip.
pub fn sphere_cuboid_collide_local(
    sphere_center: &Vector3<Float>,
    sphere_geometry: &SphereGeometry,
    cuboid_iso: &Isometry3<Float>,
    cuboid_geometry: &CuboidGeometry,
) -> Vec<Contact> {
    let half = cuboid_geometry.half_extents();
    let r = sphere_geometry.r;

    // Transform sphere center into cuboid-local space
    let local_center = cuboid_iso
        .inverse_transform_point(&Point3::from(*sphere_center))
        .coords;

    // Early-out AABB check
    if local_center.x.abs() > half.x + r
        || local_center.y.abs() > half.y + r
        || local_center.z.abs() > half.z + r
    {
        return vec![];
    }

    // Closest point on the AABB to the sphere center, which handles the
    // vertex, edge and face cases uniformly
    let closest_local = Vector3::new(
        local_center.x.clamp(-half.x, half.x),
        local_center.y.clamp(-half.y, half.y),
        local_center.z.clamp(-half.z, half.z),
    );

    let diff = closest_local - local_center;
    if diff.norm_squared() > r * r {
        return vec![];
    }

    // Back out to world frame
    let contact_world = cuboid_iso.transform_point(&Point3::from(closest_local));
    let normal_world = cuboid_iso.transform_vector(&diff);

    vec![Contact::new(
        contact_world.coords,
        UnitVector3::new_normalize(normal_world),
    )]
}

/// Collision detection between a cuboid and a point
/// Returns the contact point, and the contact normal outward from the cuboid, if
/// the point is inside it
pub fn cuboid_point_collide(
    cuboid_iso: &Isometry3<Float>,
    cuboid_geometry: &CuboidGeometry,
    point: &Vector3<Float>,
) -> Vec<Contact> {
    OrientedBox::new(cuboid_iso, cuboid_geometry)
        .point_contact(point, 0.)
        .into_iter()
        .collect()
}

#[cfg(test)]
mod tests {
    use na::{vector, Isometry3};

    use crate::{
        assert_vec_close,
        hybrid::{
            collision::sphere_cuboid::{sphere_cuboid_collide, sphere_cuboid_collide_local},
            visual::{CuboidGeometry, SphereGeometry},
        },
    };

    /// The world-frame variant agrees with the local-frame one on vertex,
    /// edge, and face contacts of a rotated cuboid
    #[test]
    fn sphere_cuboid_collide_world_matches_local() {
        let sphere = SphereGeometry { r: 1.0 };
        let cuboid = CuboidGeometry {
            w: 2.0,
            d: 1.0,
            h: 0.5,
        };
        let iso = Isometry3::new(vector![0.3, -0.2, 0.1], vector![0.4, 0.5, 0.6]);

        // Sphere centers producing face, edge, vertex contacts, and a miss
        let centers = [
            vector![0.3, -0.2, 1.2], // near top face
            vector![1.5, 0.5, 0.5],  // near an edge
            vector![1.5, 0.9, 0.6],  // near a vertex
            vector![5.0, 5.0, 5.0],  // no contact
        ];

        for center in centers.iter() {
            let local = sphere_cuboid_collide_local(center, &sphere, &iso, &cuboid);
            let world = sphere_cuboid_collide(center, &sphere, &iso, &cuboid);
            assert_eq!(local.len(), world.len(), "variants disagree at {center:?}");
            for (l, w) in local.iter().zip(world.iter()) {
                assert_vec_close!(l.point, w.point, 1e-9);
                assert_vec_close!(*l.normal, *w.normal, 1e-9);
            }
        }
    }
}
