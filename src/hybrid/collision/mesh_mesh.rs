//! Contacts between two meshes.

use na::{Isometry3, UnitVector3, Vector3};

use crate::{
    hybrid::{
        collision::shared::{
            candidate_faces, candidate_vertices, distinct_edges, edge_edge_contact,
            point_triangle_contact, push_welded, support_polygon, Aabb, CandidateFace, Contact,
            DeepContact, CONTACT_TOLERANCE, SUPPORT_DIRECTIONS,
        },
        visual::rigid_mesh::RigidMesh,
    },
    types::Float,
};

/// How closely two contact normals have to agree, as the cosine of the angle
/// between them, for their contacts to belong to the same patch.
///
/// About eight degrees, which is wide enough to gather the normals of one flat
/// patch -- they differ only by the rounding in normalizing each face -- and of
/// the gently varying ones a curved surface makes, while still splitting the two
/// walls of a groove into the separate patches they are.
const NORMAL_CLUSTER_COSINE: Float = 0.99;

/// Collision detection between two meshes
/// Returns a list of contacts in world frame, whose normals point from mesh 1 to
/// mesh 2
///
/// `iso1` and `iso2` take each mesh from its own frame to world. Everything runs
/// in mesh 1's frame: only the handful of mesh 2's faces that come near mesh 1
/// are brought into it, rather than transforming (and re-BVH-ing) either mesh,
/// the same way `mesh_halfspace_collide` moves the plane instead of the mesh.
///
/// Three kinds of contact are found, none of which covers another: a vertex of
/// mesh 2 resting on a face of mesh 1, a vertex of mesh 1 resting on a face of
/// mesh 2, and an edge of one crossing an edge of the other. Two boxes meeting
/// face to face touch only at vertices, one resting a corner on the other's
/// broad face touches at that one vertex with no vertex of the other anywhere
/// near it, and two bars laid across each other touch at neither -- only where
/// their edges cross.
///
/// Contact is a proximity test, as it is for the cuboid corners in
/// `mesh_cuboid_collide`: a feature counts once it comes within
/// `CONTACT_TOLERANCE` of the other mesh's surface. Neither mesh has the cheap
/// inside test a cuboid does, so a mesh driven clean through another in one step
/// -- far enough that no feature is left near a surface -- reports nothing. The
/// contact has to be caught on approach rather than recovered from afterwards.
///
/// Two faces meeting flat put every vertex and every edge crossing in the
/// overlap into contact at once, which is a row of the contact QP each, so the
/// contacts are reduced to the corners of the patches they form -- see
/// [`reduce_to_support_polygons`].
pub fn mesh_mesh_collide(
    mesh1: &RigidMesh,
    iso1: &Isometry3<Float>,
    mesh2: &RigidMesh,
    iso2: &Isometry3<Float>,
) -> Vec<Contact> {
    let tol = CONTACT_TOLERANCE;

    // Mesh 2's frame to mesh 1's, which is the frame the rest of this works in
    let mesh2_to_1 = iso1.inverse() * iso2;

    // Early-out: contact requires the two AABBs, inflated by tol, to overlap.
    // What they overlap in is also the only region worth looking at, since every
    // contact point is within tol of both surfaces.
    let aabb1 = Aabb::of_mesh(mesh1).inflated(tol);
    let aabb2 = Aabb::of_mesh(mesh2).transformed(&mesh2_to_1).inflated(tol);
    let Some(shared) = aabb1.intersection(&aabb2) else {
        return vec![];
    };

    // Only the faces reaching into that region can be in contact. Mesh 2's BVH
    // is in mesh 2's frame, so the region goes there to ask.
    let i_faces1 = mesh1.bvh.collect_overlapping(&shared.min, &shared.max);
    if i_faces1.is_empty() {
        return vec![];
    }
    let in_2 = shared.transformed(&mesh2_to_1.inverse());
    let i_faces2 = mesh2.bvh.collect_overlapping(&in_2.min, &in_2.max);
    if i_faces2.is_empty() {
        return vec![];
    }

    let faces1 = candidate_faces(mesh1, &i_faces1, None);
    let faces2 = candidate_faces(mesh2, &i_faces2, Some(&mesh2_to_1));

    let mut contacts = vec![];
    // A face of mesh 1 faces out of mesh 1, i.e. from mesh 1 towards mesh 2
    vertex_face_contacts(&faces2, &faces1, 1., tol, &mut contacts);
    // A face of mesh 2 faces the other way, so its normal is flipped
    vertex_face_contacts(&faces1, &faces2, -1., tol, &mut contacts);
    edge_edge_contacts(&faces1, &faces2, tol, &mut contacts);

    // Back out to world frame
    reduce_to_support_polygons(contacts)
        .into_iter()
        .map(|contact| contact.transformed(iso1))
        .collect()
}

/// The contacts of the corners of `faces` against `against`, i.e. the vertices
/// lying over one of those faces and within `tol` of it, each with that face's
/// normal
///
/// `sign` is +1 when the outward normals of `against` already point the way the
/// reported normal should, and -1 when they point back the other way.
fn vertex_face_contacts(
    faces: &[CandidateFace],
    against: &[CandidateFace],
    sign: Float,
    tol: Float,
    contacts: &mut Vec<DeepContact>,
) {
    let vertices = candidate_vertices(faces);
    let tol_sq = tol * tol;

    for face in against.iter() {
        let normal = UnitVector3::new_normalize(sign * face.normal);

        for point in vertices.iter() {
            let Some((_closest, offset)) = point_triangle_contact(point, &face.points, tol_sq)
            else {
                continue;
            };

            // `normal` is the face's own outward normal turned by `sign`, so
            // turning the offset back by `sign` measures it against the face
            // itself: a vertex below the face is one inside the other mesh
            push_welded(
                contacts,
                DeepContact {
                    contact: Contact::new(*point, normal),
                    depth: -sign * offset.dot(&normal),
                },
            );
        }
    }
}

/// The contacts of the first mesh's edges against the second's, i.e. the pairs
/// of edges crossing within `tol` of each other, contacting halfway across the
/// gap -- or the overlap -- between them, along their common perpendicular
///
/// The normal points out of the first mesh, i.e. from it towards the second.
fn edge_edge_contacts(
    faces1: &[CandidateFace],
    faces2: &[CandidateFace],
    tol: Float,
    contacts: &mut Vec<DeepContact>,
) {
    let tol_sq = tol * tol;
    let edges1 = distinct_edges(faces1);
    let edges2 = distinct_edges(faces2);

    for edge1 in edges1.iter() {
        let [a, b] = &edge1.ends;
        for edge2 in edges2.iter() {
            let [c, d] = &edge2.ends;
            let Some(found) = edge_edge_contact((a, b), &edge1.outward, (c, d), tol_sq) else {
                continue;
            };

            push_welded(contacts, found);
        }
    }
}

/// Cut each patch of contacts down to the corners of the polygon supporting it
///
/// The contacts are grouped by normal first. A single patch would do for two
/// meshes meeting on one pair of faces, but a mesh sitting in a groove or a
/// corner of another rests on two or three faces at once, and reducing those
/// together would keep the corners of the union and drop whole faces' worth of
/// contact -- the ones holding the body sideways. Each group is reduced in its
/// own tangent plane by [`support_polygon`], so every face that is in contact
/// keeps its own support polygon.
fn reduce_to_support_polygons(contacts: Vec<DeepContact>) -> Vec<Contact> {
    // Too few to be worth grouping: nothing can be dropped anyway
    if contacts.len() <= SUPPORT_DIRECTIONS + 1 {
        return contacts.into_iter().map(|held| held.contact).collect();
    }

    // Group by normal. The running sum of a group's normals is what a candidate
    // is compared against, rather than the first normal to land in it, so that
    // the direction a group stands for is the middle of the ones it holds.
    let mut patches: Vec<(Vector3<Float>, Vec<usize>)> = vec![];
    for (i, held) in contacts.iter().enumerate() {
        let found = patches
            .iter_mut()
            .find(|(sum, _)| sum.dot(&held.contact.normal) > NORMAL_CLUSTER_COSINE * sum.norm());
        match found {
            Some((sum, members)) => {
                *sum += held.contact.normal.into_inner();
                members.push(i);
            }
            None => patches.push((held.contact.normal.into_inner(), vec![i])),
        }
    }

    let mut reduced = vec![];
    for (sum, members) in patches.iter() {
        if members.len() <= SUPPORT_DIRECTIONS + 1 {
            reduced.extend(members.iter().map(|&i| contacts[i].contact));
            continue;
        }

        let patch = members
            .iter()
            .map(|&i| (i, contacts[i].contact.point, contacts[i].depth));
        let normal = UnitVector3::new_normalize(*sum);
        reduced.extend(
            support_polygon(&normal, patch)
                .into_iter()
                .map(|i| contacts[i].contact),
        );
    }

    reduced
}

#[cfg(test)]
mod tests {
    use na::{vector, Isometry3, Point3, Translation3, UnitQuaternion, Vector3};

    use crate::{
        assert_vec_close,
        hybrid::{
            collision::{mesh_mesh::mesh_mesh_collide, shared::SUPPORT_DIRECTIONS},
            visual::rigid_mesh::RigidMesh,
        },
        types::Float,
        PI,
    };

    /// A box mesh of the given full extents, centred on its own origin, wound
    /// so that every face normal points out of it
    fn box_mesh(w: Float, d: Float, h: Float) -> RigidMesh {
        let (x, y, z) = (w / 2., d / 2., h / 2.);
        let obj = format!(
            "v {mx} {my} {mz}\nv {x} {my} {mz}\nv {x} {y} {mz}\nv {mx} {y} {mz}\n\
             v {mx} {my} {z}\nv {x} {my} {z}\nv {x} {y} {z}\nv {mx} {y} {z}\n\
             f 1 3 2\nf 1 4 3\nf 5 6 7\nf 5 7 8\nf 1 2 6\nf 1 6 5\n\
             f 2 3 7\nf 2 7 6\nf 3 4 8\nf 3 8 7\nf 4 1 5\nf 4 5 8\n",
            mx = -x,
            my = -y,
            mz = -z
        );

        RigidMesh::new_from_obj(&obj)
    }

    /// A pyramid on a 0.4 m square base, its apex 1 m up, wound outward
    fn spike_mesh() -> RigidMesh {
        RigidMesh::new_from_obj(
            "v 0 0 1\nv -0.2 -0.2 0\nv 0.2 -0.2 0\nv 0.2 0.2 0\nv -0.2 0.2 0\n\
             f 1 2 3\nf 1 3 4\nf 1 4 5\nf 1 5 2\nf 2 3 4\nf 2 4 5\n",
        )
    }

    /// A mesh of flat square plates, each subdivided into an `n` by `n` grid of
    /// quads and wound so that its normals point along +z before `iso` puts it
    /// in place
    ///
    /// One face of a mesh only ever holds a handful of contacts, so it takes a
    /// tessellated one -- which is what a mesh off a CAD or scan pipeline is --
    /// for a flat contact to report the hundreds the reduction exists to cut.
    fn plates_mesh(plates: &[(Float, usize, Isometry3<Float>)]) -> RigidMesh {
        let mut vertices = String::new();
        let mut faces = String::new();
        let mut offset = 0;
        for &(size, n, iso) in plates.iter() {
            let at = |k: usize| size * (k as Float / n as Float - 0.5);
            for i in 0..=n {
                for j in 0..=n {
                    let p = iso * Point3::new(at(i), at(j), 0.);
                    vertices += &format!("v {} {} {}\n", p.x, p.y, p.z);
                }
            }

            // Wound anticlockwise seen from +z, so that both triangles of each
            // quad face the way the plate does
            let corner = |i: usize, j: usize| offset + i * (n + 1) + j + 1;
            for i in 0..n {
                for j in 0..n {
                    let (a, b) = (corner(i, j), corner(i + 1, j));
                    let (c, d) = (corner(i + 1, j + 1), corner(i, j + 1));
                    faces += &format!("f {a} {b} {c}\nf {a} {c} {d}\n");
                }
            }
            offset += (n + 1) * (n + 1);
        }

        RigidMesh::new_from_obj(&(vertices + &faces))
    }

    /// A mesh vertex reaching a face of the other mesh contacts it, with the
    /// normal running from the first mesh to the second, reported in world frame
    #[test]
    fn mesh_mesh_collide_finds_the_vertex_on_a_face() {
        let spike = spike_mesh();
        // A slab far wider than the pyramid, so that none of its own corners
        // come anywhere near the mesh: only the apex is in contact
        let slab = box_mesh(4., 4., 1.);

        // The apex off the diagonal the slab's bottom face is split along, so
        // that it lands squarely inside one triangle of it
        let spike_iso = Isometry3::translation(0.5, 0., 0.);
        let apex = vector![0.5, 0., 1.];

        // Bottom face at z = 1, i.e. resting on the apex
        let touching = Isometry3::translation(0., 0., 1.5);
        let contacts = mesh_mesh_collide(&spike, &spike_iso, &slab, &touching);
        assert_eq!(contacts.len(), 1);
        assert_vec_close!(contacts[0].point, apex, 1e-9);
        // Out of the spike and into the slab, i.e. the way the slab is pushed
        assert_vec_close!(*contacts[0].normal, vector![0., 0., 1.], 1e-9);

        // The apex just through the face still counts
        let sunk = Isometry3::translation(0., 0., 1.4995);
        assert_eq!(mesh_mesh_collide(&spike, &spike_iso, &slab, &sunk).len(), 1);

        // Clear of the apex, nothing
        let clear = Isometry3::translation(0., 0., 1.51);
        assert!(mesh_mesh_collide(&spike, &spike_iso, &slab, &clear).is_empty());

        // Nowhere near, nothing
        let away = Isometry3::translation(20., 0., 1.5);
        assert!(mesh_mesh_collide(&spike, &spike_iso, &slab, &away).is_empty());

        // The normal runs the other way round when the meshes are swapped
        let contacts = mesh_mesh_collide(&slab, &touching, &spike, &spike_iso);
        assert_eq!(contacts.len(), 1);
        assert_vec_close!(contacts[0].point, apex, 1e-9);
        assert_vec_close!(*contacts[0].normal, vector![0., 0., -1.], 1e-9);
    }

    /// The contact is the same one wherever in the world the pair sits: moving
    /// both meshes together carries the contact point and normal along with them
    #[test]
    fn mesh_mesh_collide_reports_contacts_in_world_frame() {
        let spike = spike_mesh();
        let slab = box_mesh(4., 4., 1.);
        let spike_iso = Isometry3::translation(0.5, 0., 0.);
        let slab_iso = Isometry3::translation(0., 0., 1.5);

        // Both meshes turned a quarter turn about x and moved off the origin
        let world = Isometry3::from_parts(
            Translation3::new(-3., 7., 2.),
            UnitQuaternion::from_axis_angle(&Vector3::x_axis(), PI / 2.),
        );
        let contacts = mesh_mesh_collide(&spike, &(world * spike_iso), &slab, &(world * slab_iso));
        assert_eq!(contacts.len(), 1);
        assert_vec_close!(
            contacts[0].point,
            (world * Point3::new(0.5, 0., 1.)).coords,
            1e-9
        );
        assert_vec_close!(*contacts[0].normal, world.rotation * Vector3::z(), 1e-9);
    }

    /// Two bars laid across each other touch only where their edges cross:
    /// neither one has a vertex over a face of the other
    #[test]
    fn mesh_mesh_collide_finds_the_edge_edge_points() {
        // A bar along x, its top face at z = 0.05
        let along_x = box_mesh(2., 0.1, 0.1);
        // A bar along y, laid across it with its bottom face 0.5 mm through
        let along_y = box_mesh(0.1, 2., 0.1);
        let across = Isometry3::translation(0., 0., 0.0995);

        let contacts = mesh_mesh_collide(&along_x, &Isometry3::identity(), &along_y, &across);

        // The four crossings of the bars' long top and bottom edges, plus the
        // five the diagonals the two faces are triangulated along make: each
        // diagonal crosses the other bar's two long edges, and they cross each
        // other over the middle
        assert_eq!(contacts.len(), 9);
        for corner in [
            vector![0.05, 0.05, 0.04975],
            vector![0.05, -0.05, 0.04975],
            vector![-0.05, 0.05, 0.04975],
            vector![-0.05, -0.05, 0.04975],
        ] {
            assert!(
                contacts.iter().any(|c| (c.point - corner).norm() < 1e-9),
                "no contact at {corner:?}"
            );
        }
        // Every one of them pushes the upper bar up
        for contact in contacts.iter() {
            assert_vec_close!(*contact.normal, vector![0., 0., 1.], 1e-9);
        }

        // Lifted clear of the lower bar, nothing
        let lifted = Isometry3::translation(0., 0., 0.12);
        assert!(mesh_mesh_collide(&along_x, &Isometry3::identity(), &along_y, &lifted).is_empty());
    }

    /// A tessellated plate landing flat on a face touches it at every one of its
    /// vertices, which is far more contact than the patch needs; what comes back
    /// is the corners of that patch, and the whole of it, not just the middle
    #[test]
    fn mesh_mesh_collide_reduces_a_flat_patch_to_its_support_polygon() {
        // 121 vertices, every one of them landing on the slab
        let plate = plates_mesh(&[(1., 10, Isometry3::identity())]);
        // Far wider than the plate, so that none of its own corners are near it
        let slab = box_mesh(4., 4., 1.);

        // The plate resting on the slab's top face at z = 0.5
        let contacts = mesh_mesh_collide(
            &slab,
            &Isometry3::identity(),
            &plate,
            &Isometry3::translation(0., 0., 0.5),
        );

        assert!(!contacts.is_empty());
        assert!(
            contacts.len() <= SUPPORT_DIRECTIONS + 1,
            "{} contacts survived the reduction",
            contacts.len()
        );

        // All one patch, so all one normal: out of the slab and into the plate
        for contact in contacts.iter() {
            assert_vec_close!(*contact.normal, vector![0., 0., 1.], 1e-9);
        }

        // The corners of the plate are what stops it rocking, so they are the
        // contacts that have to survive
        for corner in [
            vector![0.5, 0.5, 0.5],
            vector![0.5, -0.5, 0.5],
            vector![-0.5, 0.5, 0.5],
            vector![-0.5, -0.5, 0.5],
        ] {
            assert!(
                contacts.iter().any(|c| (c.point - corner).norm() < 1e-9),
                "no contact at {corner:?}"
            );
        }
    }

    /// A mesh in contact on two faces at once is reduced a patch at a time, so
    /// the face holding it sideways survives the face holding it up
    #[test]
    fn mesh_mesh_collide_reduces_each_contact_patch_separately() {
        // A floor at z = 0 and a wall at x = -0.5 facing along +x, both
        // tessellated finely enough to swamp either patch on their own
        let corner = plates_mesh(&[
            (2., 6, Isometry3::identity()),
            (
                2.,
                6,
                Isometry3::from_parts(
                    Translation3::new(-0.5, 0., 0.5),
                    UnitQuaternion::from_axis_angle(&Vector3::y_axis(), PI / 2.),
                ),
            ),
        ]);

        // A box filling the corner: its underside on the floor, its left face
        // against the wall
        let block = box_mesh(1., 2., 1.);
        let contacts = mesh_mesh_collide(
            &corner,
            &Isometry3::identity(),
            &block,
            &Isometry3::translation(0., 0., 0.5),
        );

        // Reduced, but to a support polygon per patch rather than one overall
        assert!(
            contacts.len() <= 2 * (SUPPORT_DIRECTIONS + 1),
            "{} contacts survived the reduction",
            contacts.len()
        );
        for held in [vector![0., 0., 1.], vector![1., 0., 0.]] {
            assert!(
                contacts.iter().any(|c| (*c.normal - held).norm() < 1e-6),
                "nothing left holding the block along {held:?}"
            );
        }
    }
}
