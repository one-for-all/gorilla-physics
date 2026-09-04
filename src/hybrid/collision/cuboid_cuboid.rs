//! Contacts between two cuboids, by the separating-axis test.

use na::{Isometry3, UnitVector3, Vector3};

use crate::{
    hybrid::{
        collision::{
            oriented_box::OrientedBox,
            shared::{clip_polygon, segment_closest_points, Contact, CONTACT_TOLERANCE},
        },
        visual::CuboidGeometry,
    },
    types::Float,
};

/// How much a face axis is favoured over an edge-edge axis of the same depth.
///
/// A cuboid resting squarely on another has a face axis and several edge-edge
/// axes overlapping by exactly the same amount, and rounding alone decides
/// between them. Only the face axis yields a polygon of contact points rather
/// than a single one, so it has to win those ties.
const EDGE_AXIS_BIAS: Float = 1e-6;

/// Which pair of features the separating-axis test settled on.
#[derive(Clone, Copy)]
enum SatFeature {
    /// A face of the first cuboid
    Face1,
    /// A face of the second cuboid
    Face2,
    /// An edge of the first cuboid along its axis `i`, against an edge of the
    /// second along its axis `j`
    Edge(usize, usize),
}

/// Collision detection between two cuboids
/// Returns a list of contacts whose normals point from cuboid 1 to cuboid 2
///
/// The normal is the axis of least overlap found by the separating-axis test
/// over the 15 candidate axes: the three face normals of each cuboid, and the
/// nine cross products of their edge directions. Pairs nowhere near each other
/// are dropped before that, by their bounding spheres and then their AABBs.
///
/// A face contact is reported as the polygon the two facing faces share, up to
/// four points, so that a cuboid landing flat is held by its whole face instead
/// of pivoting on one corner; an edge-edge contact is the single point where the
/// two edges come closest.
pub fn cuboid_cuboid_collide(
    iso1: &Isometry3<Float>,
    geometry1: &CuboidGeometry,
    iso2: &Isometry3<Float>,
    geometry2: &CuboidGeometry,
) -> Vec<Contact> {
    let tol = CONTACT_TOLERANCE;

    let box1 = OrientedBox::new(iso1, geometry1);
    let box2 = OrientedBox::new(iso2, geometry2);

    // Between the centers, i.e. the direction from cuboid 1 towards cuboid 2
    let d = box2.center() - box1.center();

    // Early-out: two cuboids too far apart to touch are the common case -- most
    // pairs in a scene never come near each other -- so cut them with a single
    // length comparison between their bounding spheres, before the axes below.
    let reach = box1.half.norm() + box2.half.norm() + tol;
    if d.norm_squared() > reach * reach {
        return vec![];
    }

    // Early-out: and then with the world axes, which the 15 candidates below do
    // not cover. This is the overlap test of the two world-frame AABBs.
    if !box1.aabb().inflated(tol).overlaps(&box2.aabb()) {
        return vec![];
    }

    // The three face normals of each cuboid, then the nine cross products of
    // their edge directions. Face axes come first: they separate far more pairs
    // than the cross products do, and the test stops at the first axis that does.
    let candidates: [(Vector3<Float>, SatFeature); 15] = core::array::from_fn(|c| match c {
        0..=2 => (box1.axes[c], SatFeature::Face1),
        3..=5 => (box2.axes[c - 3], SatFeature::Face2),
        _ => {
            let (i, j) = ((c - 6) / 3, (c - 6) % 3);
            (box1.axes[i].cross(&box2.axes[j]), SatFeature::Edge(i, j))
        }
    });

    let mut best_overlap = Float::MAX;
    let mut best_axis = Vector3::zeros();
    let mut best_feature = SatFeature::Face1;
    for (axis, feature) in candidates {
        // Parallel edges leave no cross product to test along, and the face
        // axes already cover every direction that pair could separate along
        let norm = axis.norm();
        if norm < 1e-9 {
            continue;
        }
        let axis = axis / norm;

        // How far the two cuboids reach towards each other along the axis, less
        // how far apart their centers are along it
        let overlap = box1.radius(&axis) + box2.radius(&axis) - d.dot(&axis).abs();
        if overlap < -tol {
            return vec![];
        }

        let biased = match feature {
            SatFeature::Edge(_, _) => overlap + EDGE_AXIS_BIAS,
            _ => overlap,
        };
        if biased < best_overlap {
            best_overlap = biased;
            // Orient every candidate axis from cuboid 1 towards cuboid 2
            best_axis = if d.dot(&axis) >= 0. { axis } else { -axis };
            best_feature = feature;
        }
    }

    let normal = UnitVector3::new_normalize(best_axis);
    let points = match best_feature {
        SatFeature::Face1 => face_contacts(&box1, &normal, &box2, tol),
        // The reference face is on cuboid 2, so it faces back towards cuboid 1
        SatFeature::Face2 => face_contacts(&box2, &(-normal), &box1, tol),
        SatFeature::Edge(i, j) => edge_contacts(&box1, i, &box2, j, &normal),
    };

    points
        .into_iter()
        .map(|point| Contact::new(point, normal))
        .collect()
}

/// The contact points of the reference cuboid's face pointing along `direction`
/// against the incident cuboid, i.e. the corners of the incident cuboid's facing
/// face that lie over the reference face and reach it
fn face_contacts(
    reference: &OrientedBox,
    direction: &UnitVector3<Float>,
    incident: &OrientedBox,
    tol: Float,
) -> Vec<Vector3<Float>> {
    let ref_face = reference.support_face(direction);
    // The incident face is the one turned back towards the reference face
    let inc_face = incident.support_face(&(-direction.into_inner()));

    // Cut the incident face down to the part of it lying over the reference
    // face, against the reference face's four side planes
    let mut polygon = inc_face.corners().to_vec();
    for side in ref_face.sides.iter() {
        for sign in [1., -1.] {
            let normal = sign * side;
            polygon = clip_polygon(&polygon, &normal, &(ref_face.center + normal));
            if polygon.is_empty() {
                return vec![];
            }
        }
    }

    // Keep the corners that reach the reference face, and put the contact
    // halfway across the gap -- or the overlap -- so that it favours neither
    // cuboid's surface
    polygon
        .into_iter()
        .filter_map(|p| {
            let depth = (p - ref_face.center).dot(&ref_face.normal);
            (depth <= tol).then(|| p - 0.5 * depth * ref_face.normal)
        })
        .collect()
}

/// The single contact point where the two cuboids' closest edges, along the
/// given axis of each, come nearest to each other
fn edge_contacts(
    box1: &OrientedBox,
    i1: usize,
    box2: &OrientedBox,
    i2: usize,
    normal: &UnitVector3<Float>,
) -> Vec<Vector3<Float>> {
    let edge1 = box1.support_edge(i1, &normal.into_inner());
    let edge2 = box2.support_edge(i2, &(-normal.into_inner()));
    let (p1, p2) = segment_closest_points(&edge1[0], &edge1[1], &edge2[0], &edge2[1]);

    // Halfway across the gap -- or the overlap -- between the two edges
    vec![(p1 + p2) / 2.]
}

#[cfg(test)]
mod tests {
    use na::{vector, Isometry3, Translation3, UnitQuaternion, Vector3};

    use crate::{
        assert_vec_close,
        hybrid::{collision::cuboid_cuboid::cuboid_cuboid_collide, visual::CuboidGeometry},
        types::Float,
        PI,
    };

    /// A cuboid resting flat on another is held by the four corners of the face
    /// it lands on, and the normal is the face normal
    #[test]
    fn cuboid_cuboid_collide_rests_flat_on_four_corners() {
        let ground = CuboidGeometry {
            w: 4.,
            d: 4.,
            h: 1.,
        };
        let ground_iso = Isometry3::translation(0., 0., -0.5);
        let cube = CuboidGeometry {
            w: 1.,
            d: 1.,
            h: 1.,
        };
        let cube_iso = Isometry3::translation(0., 0., 0.5);

        let contacts = cuboid_cuboid_collide(&ground_iso, &ground, &cube_iso, &cube);
        assert_eq!(contacts.len(), 4);
        for contact in contacts.iter() {
            // On the shared face, at the corners of the smaller cuboid
            assert!(contact.point.z.abs() < 1e-9);
            assert!((contact.point.x.abs() - 0.5).abs() < 1e-9);
            assert!((contact.point.y.abs() - 0.5).abs() < 1e-9);
            // Pointing from the ground up to the cuboid resting on it
            assert_vec_close!(*contact.normal, vector![0., 0., 1.], 1e-9);
        }

        // Held the same way once the two overlap
        let sunk = Isometry3::translation(0., 0., 0.5 - 1e-2);
        assert_eq!(
            cuboid_cuboid_collide(&ground_iso, &ground, &sunk, &cube).len(),
            4
        );

        // And not at all once it clears the ground
        let clear = Isometry3::translation(0., 0., 1.);
        assert!(cuboid_cuboid_collide(&ground_iso, &ground, &clear, &cube).is_empty());

        // Nor when it is alongside rather than above
        let beside = Isometry3::translation(3., 0., 0.5);
        assert!(cuboid_cuboid_collide(&ground_iso, &ground, &beside, &cube).is_empty());
    }

    /// Two cuboids meeting edge to edge contact at the single point where the
    /// edges cross, with the normal along the cross product of the two edges
    #[test]
    fn cuboid_cuboid_collide_finds_the_edge_edge_point() {
        let cube = CuboidGeometry {
            w: 1.,
            d: 1.,
            h: 1.,
        };

        // A cube tipped 45 degrees about x, so that its lowest feature is the
        // edge running along x, hanging sqrt(2)/2 below its center
        let tipped = Isometry3::from_parts(
            Translation3::new(0., 0., (2. as Float).sqrt() / 2.),
            UnitQuaternion::from_axis_angle(&Vector3::x_axis(), PI / 4.),
        );
        // And another tipped the same way about y, so its lowest edge runs along
        // y: the two cross at the origin
        let crossing = Isometry3::from_parts(
            Translation3::new(0., 0., -(2. as Float).sqrt() / 2.),
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), PI / 4.),
        );

        let contacts = cuboid_cuboid_collide(&crossing, &cube, &tipped, &cube);
        assert_eq!(contacts.len(), 1);
        assert_vec_close!(contacts[0].point, Vector3::<Float>::zeros(), 1e-9);
        assert_vec_close!(*contacts[0].normal, vector![0., 0., 1.], 1e-9);

        // Sliding the upper cube along its own edge leaves the two edges
        // crossing where they did
        let along = tipped * Translation3::new(0.25, 0., 0.);
        let contacts = cuboid_cuboid_collide(&crossing, &cube, &along, &cube);
        assert_eq!(contacts.len(), 1);
        assert_vec_close!(contacts[0].point, Vector3::<Float>::zeros(), 1e-9);

        // Lifting it off separates them
        let lifted = Translation3::new(0., 0., 0.1) * tipped;
        assert!(cuboid_cuboid_collide(&crossing, &cube, &lifted, &cube).is_empty());
    }

    /// The contact points of two cuboids overlapping only partly are the corners
    /// of the overlap, whichever cuboid's face they come from
    #[test]
    fn cuboid_cuboid_collide_clips_to_the_shared_face() {
        let cube = CuboidGeometry {
            w: 1.,
            d: 1.,
            h: 1.,
        };
        let lower = Isometry3::translation(0., 0., 0.);
        // Overlapping the lower cube's top face over a quarter of it
        let upper = Isometry3::translation(0.5, 0.5, 1.);

        let contacts = cuboid_cuboid_collide(&lower, &cube, &upper, &cube);
        assert_eq!(contacts.len(), 4);
        for contact in contacts.iter() {
            assert!((contact.point.z - 0.5).abs() < 1e-9);
            // The overlap runs from 0 to 0.5 along both x and y
            assert!(contact.point.x > -1e-9 && contact.point.x < 0.5 + 1e-9);
            assert!(contact.point.y > -1e-9 && contact.point.y < 0.5 + 1e-9);
            assert_vec_close!(*contact.normal, vector![0., 0., 1.], 1e-9);
        }
    }
}
