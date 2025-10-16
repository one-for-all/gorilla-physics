import { InterfaceHybrid } from "gorilla-physics";
import {
  BufferAttribute,
  BufferGeometry,
  DoubleSide,
  EdgesGeometry,
  LineBasicMaterial,
  LineSegments,
  Matrix4,
  Mesh,
  MeshPhongMaterial,
  Points,
  PointsMaterial,
  WireframeGeometry,
} from "three";
import { Simulator } from "./Simulator";

declare module "./Simulator" {
  interface Simulator {
    addHybrid(state: InterfaceHybrid): void;
    updateHybrid(): void;
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
        let visual_offset = new Matrix4().makeTranslation(
          iso[4],
          iso[5],
          iso[6],
        );
        if (state.visual_type(i, j, k) == 0) {
          let r = state.visual_sphere_r(i, j, k);
          this.addSphere(frame + "-" + k, 0xff0000, r, visual_offset);
        } else if (state.visual_type(i, j, k) == 1) {
          let wdh = state.visual_cuboid_wdh(i, j, k);
          this.addCuboid(
            frame + "-" + k,
            0xff0000,
            wdh[0],
            wdh[1],
            wdh[2],
            visual_offset,
          );
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
    offset += face_n;
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
      opacity: 0.5,
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
};
