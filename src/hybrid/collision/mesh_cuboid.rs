//! Contacts between a mesh and a cuboid.

use na::{Isometry3, UnitVector3, Vector3};

use crate::{
    hybrid::{
        collision::{
            oriented_box::OrientedBox,
            shared::{
                candidate_faces, candidate_vertices, distinct_edges, edge_edge_contact,
                point_triangle_contact, push_welded, Aabb, CandidateFace, Contact,
                CONTACT_TOLERANCE,
            },
        },
        visual::{rigid_mesh::RigidMesh, CuboidGeometry},
    },
    types::Float,
};

/// Collision detection between a mesh and a cuboid
/// Returns a list of contacts whose normals point from the mesh to the cuboid
///
/// Three kinds of contact are found: a corner of the cuboid resting on a mesh
/// face, a vertex of the mesh poking into a cuboid face, and an edge of the
/// cuboid crossing an edge of the mesh. No pass covers another -- a cuboid can
/// swallow the tip of a spike with every one of its own corners far from the
/// mesh, it can rest a corner on a broad mesh face with no mesh vertex anywhere
/// inside it, and it can lie across a ridge of the mesh with neither a corner
/// on a face nor a vertex inside it.
pub fn mesh_cuboid_collide(
    mesh: &RigidMesh,
    cuboid_iso: &Isometry3<Float>,
    cuboid_geometry: &CuboidGeometry,
) -> Vec<Contact> {
    let tol = CONTACT_TOLERANCE;
    let cuboid = OrientedBox::new(cuboid_iso, cuboid_geometry);

    // Early-out: contact requires the cuboid to come within tol of the mesh, so
    // the cuboid's world-frame AABB, inflated by tol, must overlap the mesh's
    let query = cuboid.aabb().inflated(tol);
    if !query.overlaps(&Aabb::of_mesh(mesh)) {
        return vec![];
    }

    // The mesh faces reaching the cuboid, which is what both the vertex pass and
    // the edge pass work over. The corner pass asks for a narrower set of its
    // own, since only the corners actually near the mesh matter to it.
    let i_faces = mesh.bvh.collect_overlapping(&query.min, &query.max);
    let faces = candidate_faces(mesh, &i_faces, None);

    let mut contacts = cuboid_corner_mesh_face_contacts(mesh, &cuboid, tol);
    contacts.extend(cuboid_face_mesh_vertex_contacts(&cuboid, &faces, tol));
    contacts.extend(cuboid_edge_mesh_edge_contacts(&cuboid, &faces, tol));

    contacts
}

/// The contacts of the cuboid's corners against the mesh's faces, i.e. the
/// corners lying over a mesh face and within `tol` of it, each with that face's
/// outward normal
fn cuboid_corner_mesh_face_contacts(
    mesh: &RigidMesh,
    cuboid: &OrientedBox,
    tol: Float,
) -> Vec<Contact> {
    // Only corners within tol of the mesh AABB can contact a face; filter the
    // rest out once instead of testing them against every face
    let mesh_aabb = Aabb::of_mesh(mesh);
    let tol_sq = tol * tol;
    let corners: Vec<Vector3<Float>> = cuboid
        .corners()
        .into_iter()
        .filter(|p| mesh_aabb.distance_sq_to(p) <= tol_sq)
        .collect();

    // Only faces whose AABB overlaps the tol-inflated AABB of the surviving
    // corners can be in contact
    let Some(query) = Aabb::of_points(corners.iter().copied()) else {
        return vec![];
    };
    let query = query.inflated(tol);
    let i_faces = mesh.bvh.collect_overlapping(&query.min, &query.max);

    let mut contacts = vec![];
    for face in candidate_faces(mesh, &i_faces, None).iter() {
        let normal = UnitVector3::new_normalize(face.normal); // outward normal of the face
        for corner in corners.iter() {
            if point_triangle_contact(corner, &face.points, tol_sq).is_some() {
                contacts.push(Contact::new(*corner, normal));
            }
        }
    }

    contacts
}

/// The contacts of the mesh's vertices against the cuboid's faces, i.e. the
/// mesh vertices inside the cuboid -- or within `tol` of its surface -- each
/// pushed back out through the face of the cuboid nearest to it
fn cuboid_face_mesh_vertex_contacts(
    cuboid: &OrientedBox,
    faces: &[CandidateFace],
    tol: Float,
) -> Vec<Contact> {
    let mut contacts = vec![];
    for vertex in candidate_vertices(faces).iter() {
        let Some(outward) = cuboid.point_contact(vertex, tol) else {
            continue;
        };

        // The normal points out of the cuboid, hence towards the mesh; the
        // contact normal runs the other way, from the mesh to the cuboid
        push_welded(&mut contacts, outward.flipped());
    }

    contacts
}

/// The contacts of the cuboid's edges against the mesh's edges, i.e. the pairs
/// of edges crossing within `tol` of each other, contacting halfway across the
/// gap -- or the overlap -- between them, along their common perpendicular
fn cuboid_edge_mesh_edge_contacts(
    cuboid: &OrientedBox,
    faces: &[CandidateFace],
    tol: Float,
) -> Vec<Contact> {
    let cuboid_edges = cuboid.edges();
    let tol_sq = tol * tol;

    // The mesh edge comes first, so that the normal `edge_edge_contact` reports
    // runs out of the mesh, i.e. from the mesh to the cuboid
    let mut contacts = vec![];
    for mesh_edge in distinct_edges(faces).iter() {
        let [a, b] = &mesh_edge.ends;
        for edge in cuboid_edges.iter() {
            let Some(found) =
                edge_edge_contact((a, b), &mesh_edge.outward, (&edge[0], &edge[1]), tol_sq)
            else {
                continue;
            };

            push_welded(&mut contacts, found.contact);
        }
    }

    contacts
}

#[cfg(test)]
mod tests {
    use na::{vector, Isometry3, Translation3, UnitQuaternion, Vector3};

    use crate::{
        assert_vec_close,
        hybrid::{
            collision::mesh_cuboid::mesh_cuboid_collide,
            visual::{rigid_mesh::RigidMesh, CuboidGeometry},
        },
        types::Float,
        PI,
    };

    /// A mesh vertex poking into a cuboid face contacts it, even with every
    /// corner of the cuboid far from the mesh
    #[test]
    fn mesh_cuboid_collide_finds_the_mesh_vertex_in_a_cuboid_face() {
        // A pyramid on a 0.4 m square base, its apex 1 m up
        let spike = RigidMesh::new_from_obj(
            "v 0 0 1\nv -0.2 -0.2 0\nv 0.2 -0.2 0\nv 0.2 0.2 0\nv -0.2 0.2 0\n\
             f 1 2 3\nf 1 3 4\nf 1 4 5\nf 1 5 2\nf 2 3 4\nf 2 4 5\n",
        );
        // A slab far wider than the pyramid, so that none of its own corners
        // come anywhere near the mesh: only the apex is in contact
        let slab = CuboidGeometry {
            w: 4.,
            d: 4.,
            h: 1.,
        };

        // Bottom face at z = 0.9, with the apex 0.1 m through it
        let over = Isometry3::translation(0., 0., 1.4);
        let contacts = mesh_cuboid_collide(&spike, &over, &slab);
        assert_eq!(contacts.len(), 1);
        assert_vec_close!(contacts[0].point, vector![0., 0., 1.], 1e-9);
        // Out of the mesh and into the slab, i.e. the way the slab is pushed
        assert_vec_close!(*contacts[0].normal, vector![0., 0., 1.], 1e-9);

        // The apex touching the bottom face still counts
        let touching = Isometry3::translation(0., 0., 1.5);
        assert_eq!(mesh_cuboid_collide(&spike, &touching, &slab).len(), 1);

        // Clear of the apex, nothing
        let clear = Isometry3::translation(0., 0., 1.6);
        assert!(mesh_cuboid_collide(&spike, &clear, &slab).is_empty());

        // Swallowed whole, the apex leaves through the face it is nearest to,
        // which is now the slab's -x side rather than its bottom: the slab's
        // own x half extent of 2 puts that face 0.1 m away, its bottom 0.5 m
        let around = Isometry3::translation(1.9, 0., 1.);
        let contacts = mesh_cuboid_collide(&spike, &around, &slab);
        let apex = contacts
            .iter()
            .find(|c| (c.point - vector![0., 0., 1.]).norm() < 1e-9)
            .expect("the apex is inside the slab");
        assert_vec_close!(*apex.normal, vector![1., 0., 0.], 1e-9);
    }

    /// A cuboid edge laid across a ridge of the mesh contacts it at the point
    /// where the two edges cross, with neither a cuboid corner on a mesh face
    /// nor a mesh vertex inside the cuboid
    #[test]
    fn mesh_cuboid_collide_finds_the_edge_edge_point() {
        // A roof: a triangular prism whose ridge runs along x at z = 1, from
        // x = -1 to x = 1, over a base 2 m square in the z = 0 plane
        let roof = RigidMesh::new_from_obj(
            "v -1 0 1\nv 1 0 1\nv -1 -1 0\nv 1 -1 0\nv -1 1 0\nv 1 1 0\n\
             f 1 3 4\nf 1 4 2\nf 2 6 5\nf 2 5 1\nf 3 6 4\nf 3 5 6\n\
             f 2 4 6\nf 1 5 3\n",
        );
        let cube = CuboidGeometry {
            w: 1.,
            d: 1.,
            h: 1.,
        };

        // The cube tipped 45 degrees about y, so that its lowest feature is the
        // edge running along y, hanging sqrt(2)/2 below its center: laid at
        // z = 1 it crosses the ridge at right angles, over the origin
        let sqrt2 = (2. as Float).sqrt();
        let across = Isometry3::from_parts(
            Translation3::new(0., 0., 1. + sqrt2 / 2.),
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), PI / 4.),
        );

        let contacts = mesh_cuboid_collide(&roof, &across, &cube);
        assert_eq!(contacts.len(), 1);
        assert_vec_close!(contacts[0].point, vector![0., 0., 1.], 1e-9);
        // Out of the mesh and into the cube, i.e. the way the cube is pushed
        assert_vec_close!(*contacts[0].normal, vector![0., 0., 1.], 1e-9);

        // Sliding the cube along its own edge leaves the crossing where it was
        let along = Translation3::new(0., 0.25, 0.) * across;
        let contacts = mesh_cuboid_collide(&roof, &along, &cube);
        assert_eq!(contacts.len(), 1);
        assert_vec_close!(contacts[0].point, vector![0., 0., 1.], 1e-9);

        // Sliding it along the ridge carries the crossing with it
        let over = Translation3::new(0.25, 0., 0.) * across;
        let contacts = mesh_cuboid_collide(&roof, &over, &cube);
        assert_eq!(contacts.len(), 1);
        assert_vec_close!(contacts[0].point, vector![0.25, 0., 1.], 1e-9);

        // Lifting it off separates them
        let lifted = Translation3::new(0., 0., 0.1) * across;
        assert!(mesh_cuboid_collide(&roof, &lifted, &cube).is_empty());
    }
}
