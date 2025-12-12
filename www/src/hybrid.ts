import { InterfaceHybrid } from "gorilla-physics";
import {
  BufferAttribute,
  BufferGeometry,
  DoubleSide,
  EdgesGeometry,
  LineBasicMaterial,
  LineSegments,
  Matrix,
  Matrix4,
  Mesh,
  MeshPhongMaterial,
  Points,
  PointsMaterial,
  Quaternion,
  Vector3,
  WireframeGeometry,
} from "three";
import { Simulator } from "./Simulator";
import { FloatArray } from "./type";

declare module "./Simulator" {
  interface Simulator {
    addHybrid(state: InterfaceHybrid): void;
    updateHybrid(): void;
    addRigidMesh(
      name: string,
      color: Float32Array,
      vertices: Float32Array,
      faces: Uint32Array,
      offset: Matrix4,
    ): void;
  }
}

Simulator.prototype.addHybrid = function (state: InterfaceHybrid) {
  this.hybrid = state;
  // add rigid body
  let n_rigid_bodies = state.n_rigid_bodies();
  for (let i = 0; i < n_rigid_bodies; i++) {
    this.addSphere("sphere " + i, 0xff0000, 1.0);
  }

  // add articulated
  let n_articulated = state.n_articulated();
  for (let i = 0; i < n_articulated; i++) {
    let n_bodies = state.n_bodies_articulated(i);
    for (let j = 0; j < n_bodies; j++) {
      let frame = state.frame_articulated_body(i, j);
      let n_visuals = state.n_visuals_articulated_body(i, j);
      for (let k = 0; k < n_visuals; k++) {
        let iso = state.iso_visual_to_body(i, j, k);
        let q = new Quaternion(iso[0], iso[1], iso[2], iso[3]);
        let t = new Vector3(iso[4], iso[5], iso[6]);
        let visual_offset = new Matrix4().compose(t, q, new Vector3(1, 1, 1)); // last arg is unit scale
        let visual_type = state.visual_type(i, j, k);
        let visual_name = frame + "-" + k;
        if (visual_type == 0) {
          let r = state.visual_sphere_r(i, j, k);
          this.addSphere(visual_name, 0xff0000, r, visual_offset);
        } else if (visual_type == 1) {
          let wdh = state.visual_cuboid_wdh(i, j, k);
          this.addCuboid(
            visual_name,
            0xff0000,
            wdh[0],
            wdh[1],
            wdh[2],
            visual_offset,
          );
        } else if (visual_type == 2) {
          let vertices = state.visual_mesh_vertices(i, j, k);
          let faces = state.visual_mesh_faces(i, j, k);
          let visual_color = state.visual_color(i, j, k);
          this.addRigidMesh(
            visual_name,
            visual_color,
            vertices,
            faces,
            visual_offset,
          );
        } else {
          alert(`unknown visual type: ${visual_type}`);
        }
      }
    }
  }

  // add deformables
  let all_nodes = state.deformable_nodes();
  let dofs = state.deformable_dofs();

  let nodes_array: Float32Array[] = [];
  let offset = 0;
  for (let i = 0; i < dofs.length; i++) {
    const dof = dofs[i];
    nodes_array.push(all_nodes.subarray(offset, offset + dof));
    offset += dof;
  }

  let all_faces = state.deformable_faces();
  let face_ns = state.deformable_face_ns();
  if (dofs.length != face_ns.length) {
    alert("dof.len != face_ns.len");
  }

  let faces_array: Uint32Array[] = [];
  offset = 0;
  for (let i = 0; i < face_ns.length; i++) {
    const face_n = face_ns[i];
    faces_array.push(all_faces.subarray(offset, offset + face_n * 3));
    offset += face_n * 3;
  }

  for (let i = 0; i < dofs.length; i++) {
    const geometry = new BufferGeometry();
    let nodes = nodes_array[i];
    geometry.setAttribute("position", new BufferAttribute(nodes, 3));
    let facets = faces_array[i];
    geometry.setIndex(new BufferAttribute(facets, 1));
    geometry.computeVertexNormals();

    const material = new MeshPhongMaterial({
      color: 0x2194ce,
      side: DoubleSide, // Render both sides of faces
      flatShading: true,
      transparent: true,
      opacity: 1.0,
    });
    const mesh = new Mesh(geometry, material);

    mesh.frustumCulled = false; // prevent mesh disappearing
    this.meshes.set("deformable " + i, mesh);
    this.graphics.scene.add(mesh);

    // --- Add vertex points ---
    const pointMaterial = new PointsMaterial({
      color: 0xff0000, // red
      size: 0.1, // adjust for your scale
    });
    const points = new Points(geometry, pointMaterial);
    this.graphics.scene.add(points);

    // // --- Add edges overlay ---
    // const wireframeGeometry = new WireframeGeometry(geometry);
    // const wireframe = new LineSegments(
    //   wireframeGeometry,
    //   new LineBasicMaterial({ color: 0x000000 }),
    // );
    // this.graphics.scene.add(wireframe);
  }

  // Add cloths
  let cloth_nodes = state.cloth_nodes();
  let cloth_dofs = state.cloth_dofs();

  let cloth_nodes_array: Float32Array[] = []; // break array into array of arrays
  let cloth_offset = 0;
  for (let i = 0; i < cloth_dofs.length; i++) {
    const dof = cloth_dofs[i];
    cloth_nodes_array.push(
      cloth_nodes.subarray(cloth_offset, cloth_offset + dof),
    );
    cloth_offset += dof;
  }

  let cloth_faces = state.cloth_faces();
  let cloth_face_ns = state.cloth_face_ns();
  if (cloth_dofs.length != cloth_face_ns.length) {
    alert("cloth_dofs.len != cloth_face_ns.len");
  }

  let cloth_faces_array: Uint32Array[] = [];
  cloth_offset = 0;
  for (let i = 0; i < cloth_face_ns.length; i++) {
    const face_n = cloth_face_ns[i];
    cloth_faces_array.push(
      cloth_faces.subarray(cloth_offset, cloth_offset + face_n * 3),
    );
    cloth_offset += face_n;
  }

  for (let i = 0; i < cloth_dofs.length; i++) {
    const geometry = new BufferGeometry();
    let nodes = cloth_nodes_array[i];
    geometry.setAttribute("position", new BufferAttribute(nodes, 3));
    let faces = cloth_faces_array[i];
    geometry.setIndex(new BufferAttribute(faces, 1));
    geometry.computeVertexNormals();

    const material = new MeshPhongMaterial({
      color: 0x2194ce,
      side: DoubleSide, // Render both sides of faces
      flatShading: true,
      transparent: true,
      opacity: 1.0,
    });
    const mesh = new Mesh(geometry, material);

    mesh.frustumCulled = false; // prevent mesh disappearing
    this.meshes.set("cloth " + i, mesh);
    this.graphics.scene.add(mesh);

    // --- Add vertex points ---
    const pointMaterial = new PointsMaterial({
      color: 0xff0000, // red
      size: 0.1, // adjust for your scale
    });
    const points = new Points(geometry, pointMaterial);
    this.graphics.scene.add(points);

    // // --- Add edges overlay ---
    // const wireframeGeometry = new WireframeGeometry(geometry);
    // const wireframe = new LineSegments(
    //   wireframeGeometry,
    //   new LineBasicMaterial({ color: 0x000000 }),
    // );
    // this.graphics.scene.add(wireframe);
  }

  // Add halfspaces
  let halfspaces = state.halfspaces();
  for (let i = 0; i < halfspaces.length; i += 4) {
    let n = new FloatArray([
      halfspaces[i],
      halfspaces[i + 1],
      halfspaces[i + 2],
    ]);
    let dist = halfspaces[i + 3];
    this.addPlane(n, dist, 10);
  }
};

Simulator.prototype.updateHybrid = function () {
  let state = this.hybrid;

  // update rigid body poses
  let poses = state.rigid_body_poses();
  let n_rigid_bodies = state.n_rigid_bodies();
  for (let i = 0; i < n_rigid_bodies; i++) {
    this.setPose("sphere " + i, poses.subarray(i * 7, i * 7 + 7));
  }

  // update articulated bodies poses
  let n_articulated = state.n_articulated();
  for (let i = 0; i < n_articulated; i++) {
    let n_bodies = state.n_bodies_articulated(i);
    for (let j = 0; j < n_bodies; j++) {
      let frame = state.frame_articulated_body(i, j);
      let pose = state.pose_articulated_body(i, j);
      let n_visuals = state.n_visuals_articulated_body(i, j);
      for (let k = 0; k < n_visuals; k++) {
        this.setPose(frame + "-" + k, pose);
      }
    }
  }

  // update deformable positions
  let all_nodes = state.deformable_nodes();
  let dofs = state.deformable_dofs();

  let nodes_array: Float32Array[] = [];
  let offset = 0;
  for (let i = 0; i < dofs.length; i++) {
    const dof = dofs[i];
    nodes_array.push(all_nodes.subarray(offset, offset + dof));
    offset += dof;
  }

  for (let i = 0; i < dofs.length; i++) {
    let deformable = this.meshes.get("deformable " + i);
    let nodes = nodes_array[i];
    deformable.geometry.setAttribute("position", new BufferAttribute(nodes, 3));
  }

  // update cloth positions
  let cloth_nodes = state.cloth_nodes();
  let cloth_dofs = state.cloth_dofs();

  let cloth_nodes_array: Float32Array[] = [];
  let cloth_offset = 0;
  for (let i = 0; i < cloth_dofs.length; i++) {
    const dof = cloth_dofs[i];
    cloth_nodes_array.push(
      cloth_nodes.subarray(cloth_offset, cloth_offset + dof),
    );
    cloth_offset += dof;
  }

  for (let i = 0; i < cloth_dofs.length; i++) {
    let cloth = this.meshes.get("cloth " + i);
    let nodes = cloth_nodes_array[i];
    cloth.geometry.setAttribute("position", new BufferAttribute(nodes, 3));
  }
};

Simulator.prototype.addRigidMesh = function (
  name: string,
  color: Float32Array,
  vertices: Float32Array,
  faces: Uint32Array,
  offset: Matrix4 = new Matrix4(),
) {
  const geometry = new BufferGeometry();
  geometry.setAttribute("position", new BufferAttribute(vertices, 3));
  geometry.applyMatrix4(offset);

  geometry.setIndex(new BufferAttribute(faces, 1));
  geometry.computeVertexNormals();

  let material = new MeshPhongMaterial({
    side: DoubleSide, // Render both sides of faces
    flatShading: true,
  });
  material.color.setRGB(color[0], color[1], color[2]);

  const mesh = new Mesh(geometry, material);
  mesh.frustumCulled = false; // prevent mesh from disappearing
  this.meshes.set(name, mesh);
  this.graphics.scene.add(mesh);
};
