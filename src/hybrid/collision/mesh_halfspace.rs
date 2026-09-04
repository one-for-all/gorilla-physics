//! Contacts between a mesh and a halfspace.

use na::{Isometry3, Point3, UnitVector3, Vector3};

use crate::{
    collision::halfspace::HalfSpace,
    hybrid::{
        collision::shared::{push_welded, support_polygon, Contact, HALFSPACE_CONTACT_TOLERANCE},
        visual::rigid_mesh::RigidMesh,
    },
    types::Float,
};

/// Collision detection between a mesh and a halfspace.
/// Returns a list of contacts whose normals point out of the halfspace, i.e.
/// from the halfspace to the mesh.
///
/// `mesh_iso` takes the mesh from its own frame to world. Unlike the static-body
/// meshes elsewhere, a mesh hanging off an articulated body moves every step, so
/// the halfspace is brought into the mesh's frame rather than transforming (and
/// re-BVH-ing) the mesh into the world's.
///
/// A vertex counts as touching once it is within `HALFSPACE_CONTACT_TOLERANCE`
/// of the plane, and the contacts are reduced to the corners of the patch the
/// mesh rests on, at most `SUPPORT_DIRECTIONS + 1` of them. Both exist so that a
/// body resting on a face is held by that whole face, rather than by whichever
/// vertices happen to be deepest.
pub fn mesh_halfspace_collide(
    mesh: &RigidMesh,
    mesh_iso: &Isometry3<Float>,
    halfspace: &HalfSpace,
) -> Vec<Contact> {
    let tol = HALFSPACE_CONTACT_TOLERANCE;

    // The halfspace expressed in mesh frame. Rotation preserves the norm, so the
    // transformed normal is still a unit vector.
    let normal = mesh_iso.inverse_transform_vector(&halfspace.normal);
    let point = mesh_iso
        .inverse_transform_point(&Point3::from(halfspace.point))
        .coords;

    // Early-out: the deepest the mesh can reach is the AABB corner furthest
    // along -normal, so if that corner clears the plane nothing else is inside
    let mut deepest_corner = Vector3::zeros();
    for axis in 0..3 {
        deepest_corner[axis] = if normal[axis] >= 0. {
            mesh.aabb_min[axis]
        } else {
            mesh.aabb_max[axis]
        };
    }
    if (deepest_corner - point).dot(&normal) > tol {
        return vec![];
    }

    // A mesh landing flat puts hundreds of vertices under the plane at once and
    // each one is three rows in the contact QP, so the set has to be cut down to
    // the corners of the patch it rests on -- see `support_polygon`.
    //
    // A polyhedron's deepest point below a plane is always a vertex, so testing
    // vertices alone finds every contact -- no face or edge pass needed.
    //
    // Only the vertices inside the halfspace can score, and the BVH cuts the
    // scan down to the faces reaching in: the slab a resting mesh has below the
    // plane, rather than all of its vertices, which for a robot's visual mesh is
    // tens of thousands. The faces it returns are a superset of the ones
    // reaching in, and a vertex shared by several of them is simply scored more
    // than once, so the support set is the same one the full scan finds.
    let i_faces = mesh.bvh.collect_below_plane(&normal, &point, tol);
    let inside = i_faces
        .iter()
        .flat_map(|&i_face| mesh.faces[i_face])
        .filter_map(|index| {
            let vertex = mesh.vertices[index];
            let depth = (vertex - point).dot(&normal);

            (depth <= tol).then_some((index, vertex, -depth))
        });

    // Vertices repeated at one position, as an STL-derived mesh has, land in
    // different support slots and have to be welded back together
    let mut kept: Vec<Vector3<Float>> = vec![];
    let mut contacts = vec![];
    for index in support_polygon(&UnitVector3::new_normalize(normal), inside) {
        let vertex = mesh.vertices[index];
        if push_welded(&mut kept, vertex) {
            contacts.push(Contact::new(
                (mesh_iso * Point3::from(vertex)).coords,
                halfspace.normal,
            ));
        }
    }

    contacts
}

#[cfg(test)]
mod tests {
    use na::{vector, Isometry3, Matrix3, Translation3, UnitQuaternion, Vector3};

    use crate::{
        collision::halfspace::HalfSpace,
        hybrid::{
            articulated::Articulated,
            collision::{
                mesh_halfspace::mesh_halfspace_collide, shared::HALFSPACE_CONTACT_TOLERANCE,
            },
            visual::{rigid_mesh::RigidMesh, Visual},
            Hybrid, Rigid,
        },
        inertia::SpatialInertia,
        joint::{Joint, JointPosition},
        spatial::{pose::Pose, transform::Transform3D},
        types::Float,
        PI, WORLD_FRAME,
    };

    /// A mesh contacts a halfspace on the vertices below it, reported in world
    /// frame, and only when it actually reaches the plane
    #[test]
    fn mesh_halfspace_collide_finds_vertices_below_the_plane() {
        // Unit cube centred on its own origin
        let cube = RigidMesh::new_from_obj(
            "v -0.5 -0.5 -0.5\nv 0.5 -0.5 -0.5\nv 0.5 0.5 -0.5\nv -0.5 0.5 -0.5\n\
             v -0.5 -0.5 0.5\nv 0.5 -0.5 0.5\nv 0.5 0.5 0.5\nv -0.5 0.5 0.5\n\
             f 1 2 3\nf 1 3 4\nf 5 6 7\nf 5 7 8\nf 1 2 6\nf 1 6 5\n\
             f 2 3 7\nf 2 7 6\nf 3 4 8\nf 3 8 7\nf 4 1 5\nf 4 5 8\n",
        );
        let ground = HalfSpace::new(Vector3::z_axis(), 0.);
        let tol = 1e-9;

        // Straddling the plane: the four bottom vertices are inside it
        let contacts = mesh_halfspace_collide(&cube, &Isometry3::identity(), &ground);
        assert_eq!(contacts.len(), 4);
        for contact in contacts.iter() {
            assert!((contact.point.z + 0.5).abs() < tol);
            // The contact normal is the halfspace's own, pointing out of it
            assert!((contact.normal.z - 1.).abs() < tol);
        }

        // Clear of the plane: no contact
        let above = Isometry3::translation(0., 0., 1.);
        assert!(mesh_halfspace_collide(&cube, &above, &ground).is_empty());

        // Within the contact band the face still carries the body, so that a
        // face resting flat is held by all of itself and not by the sliver of it
        // that happens to have gone through the plane
        let hovering = Isometry3::translation(0., 0., 0.5 + HALFSPACE_CONTACT_TOLERANCE / 2.);
        assert_eq!(mesh_halfspace_collide(&cube, &hovering, &ground).len(), 4);

        // Past the band, nothing
        let clear = Isometry3::translation(0., 0., 0.5 + 5. * HALFSPACE_CONTACT_TOLERANCE);
        assert!(mesh_halfspace_collide(&cube, &clear, &ground).is_empty());

        // A quarter turn about x maps the cube's -y face downwards; contacts
        // come back in world frame, so they follow the isometry
        let tipped = Isometry3::from_parts(
            Translation3::new(0., 0., -0.25),
            UnitQuaternion::from_euler_angles(PI / 2., 0., 0.),
        );
        let contacts = mesh_halfspace_collide(&cube, &tipped, &ground);
        assert_eq!(contacts.len(), 4);
        for contact in contacts.iter() {
            assert!((contact.point.z + 0.75).abs() < 1e-6);
        }
    }

    /// A face resting on the plane is held by a polygon, not by the row of
    /// vertices that happen to be deepest
    #[test]
    fn mesh_halfspace_collide_keeps_the_support_polygon_of_a_tilted_face() {
        // A 1 m square plate, finely subdivided. Ranking contacts by depth only
        // goes wrong once a face carries far more vertices than the reduction
        // keeps, which is every real robot mesh.
        let n = 10;
        let mut obj = String::new();
        for i in 0..=n {
            for j in 0..=n {
                let (x, y) = (
                    -0.5 + i as Float / n as Float,
                    -0.5 + j as Float / n as Float,
                );
                obj.push_str(&format!("v {x} {y} 0\n"));
            }
        }
        let index = |i: usize, j: usize| i * (n + 1) + j + 1; // obj indices are 1-based
        for i in 0..n {
            for j in 0..n {
                obj.push_str(&format!(
                    "f {} {} {}\n",
                    index(i, j),
                    index(i + 1, j),
                    index(i + 1, j + 1)
                ));
                obj.push_str(&format!(
                    "f {} {} {}\n",
                    index(i, j),
                    index(i + 1, j + 1),
                    index(i, j + 1)
                ));
            }
        }
        let plate = RigidMesh::new_from_obj(&obj);
        let ground = HalfSpace::new(Vector3::z_axis(), 0.);

        // Tilted by a hundredth of a degree and set just below the plane, the
        // way a body sits while it rocks itself level
        let resting = Isometry3::from_parts(
            Translation3::new(0., 0., -1e-4),
            UnitQuaternion::from_euler_angles(0.01 * PI / 180., 0., 0.),
        );
        let contacts = mesh_halfspace_collide(&plate, &resting, &ground);
        assert!(contacts.len() >= 3);

        // The contacts have to span the plate both ways. Taken by depth they
        // would all sit along its lowest edge, leaving the body free to pivot
        // about that edge and sink.
        let span = |values: Vec<Float>| {
            values.iter().cloned().fold(Float::MIN, Float::max)
                - values.iter().cloned().fold(Float::MAX, Float::min)
        };
        assert!(span(contacts.iter().map(|c| c.point.x).collect()) > 0.5);
        assert!(span(contacts.iter().map(|c| c.point.y).collect()) > 0.5);
    }

    /// A body resting on its mesh stays where it was put
    #[test]
    fn mesh_resting_on_halfspace_does_not_drift() {
        // A plate the size of the test robot's footprint, 48 x 68 mm
        let (half_x, half_y) = (0.024, 0.034);
        let n = 10;
        let mut obj = String::new();
        for i in 0..=n {
            for j in 0..=n {
                let x = -half_x + 2. * half_x * i as Float / n as Float;
                let y = -half_y + 2. * half_y * j as Float / n as Float;
                obj.push_str(&format!("v {x} {y} 0\n"));
            }
        }
        let at = |i: usize, j: usize| i * (n + 1) + j + 1;
        for i in 0..n {
            for j in 0..n {
                obj.push_str(&format!(
                    "f {} {} {}\n",
                    at(i, j),
                    at(i + 1, j),
                    at(i + 1, j + 1)
                ));
                obj.push_str(&format!(
                    "f {} {} {}\n",
                    at(i, j),
                    at(i + 1, j + 1),
                    at(i, j + 1)
                ));
            }
        }

        // Mass and inertia of the test robot's base link
        let m = 0.0157812;
        let com = vector![0.0025, -0.0256, 0.0102];
        let moment_com = Matrix3::new(
            3.62346e-06,
            7.4487e-08,
            -4.8746e-08,
            7.4487e-08,
            1.46159e-06,
            4.94826e-07,
            -4.8746e-08,
            4.94826e-07,
            4.45457e-06,
        );

        let drift_of = |com: Vector3<Float>| {
            let frame = "plate";
            let moment =
                moment_com + m * (com.norm_squared() * Matrix3::identity() - com * com.transpose());
            let mut body = Rigid::new(SpatialInertia::new(moment, m * com, m, frame));
            body.visual.push((
                Visual::RigidMesh(RigidMesh::new_from_obj(&obj)),
                Isometry3::identity(),
                None,
                None,
                true,
            ));

            let mut articulated = Articulated::new(
                vec![body],
                vec![Joint::new_floating(Transform3D::identity(
                    frame,
                    WORLD_FRAME,
                ))],
            );
            // Set down a hair above the plane, the way a dropped robot is
            articulated.set_joint_q(
                0,
                JointPosition::Pose(Pose::translation(vector![0., 0., 1e-3])),
            );

            let mut hybrid = Hybrid::empty();
            hybrid.add_halfspace(HalfSpace::new(Vector3::z_axis(), 0.));
            hybrid.add_articulated(articulated);
            for _ in 0..300 {
                hybrid.step(1. / 60., &vec![]);
            }
            hybrid.articulated[0].bodies[0].pose.translation
        };

        let centered = drift_of(Vector3::zeros());
        let offset = drift_of(com);
        println!("centred CoM: {centered:?}");
        println!("offset CoM:  {offset:?}");

        // 5 s of resting must not walk the body anywhere
        for drift in [centered, offset] {
            assert!(drift.x.abs() < 1e-4, "drifted {} m in x", drift.x);
            assert!(drift.y.abs() < 1e-4, "drifted {} m in y", drift.y);
        }
    }
}
