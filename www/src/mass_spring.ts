import { InterfaceMassSpring } from "gorilla-physics";
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
    addMassSpring(deformable: InterfaceMassSpring): void;
    updateMassSpring(): void;
  }
}

Simulator.prototype.addMassSpring = function (deformable: InterfaceMassSpring) {
  this.massSpring = deformable;

  const geometry = new BufferGeometry();
  let nodes = deformable.nodes();
  geometry.setAttribute("position", new BufferAttribute(nodes, 3));
  let facets = deformable.facets();
  geometry.setIndex(new BufferAttribute(facets, 1));
  geometry.computeVertexNormals();

  const material = new MeshPhongMaterial({
    color: 0x2194ce,
    side: DoubleSide, // Render both sides of faces
    flatShading: true,
  });
  const mesh = new Mesh(geometry, material);

  mesh.frustumCulled = false; // prevent mesh disappearing
  this.meshes.set("deformable", mesh);
  this.graphics.scene.add(mesh);
};

Simulator.prototype.updateMassSpring = function () {
  let deformable = this.meshes.get("deformable");
  let nodes = this.massSpring.nodes();
  deformable.geometry.setAttribute("position", new BufferAttribute(nodes, 3));
};
