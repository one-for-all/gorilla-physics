import { InterfaceHybrid } from "gorilla-physics";
import {
  BufferAttribute,
  BufferGeometry,
  DoubleSide,
  Mesh,
  MeshPhongMaterial,
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
    });
    const mesh = new Mesh(geometry, material);

    mesh.frustumCulled = false; // prevent mesh disappearing
    this.meshes.set("deformable " + i, mesh);
    this.graphics.scene.add(mesh);
  }
};

Simulator.prototype.updateHybrid = function () {
  let state = this.hybrid;
  let poses = state.rigid_body_poses();
  let n_rigid_bodies = state.n_rigid_bodies();
  for (let i = 0; i < n_rigid_bodies; i++) {
    this.setPose("sphere " + i, poses.subarray(i * 7, 7));
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
