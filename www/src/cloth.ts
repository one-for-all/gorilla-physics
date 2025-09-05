import { InterfaceCloth } from "gorilla-physics";
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
    addCloth(cloth: InterfaceCloth): void;
    updateCloth(): void;
  }
}

Simulator.prototype.addCloth = function (cloth: InterfaceCloth) {
  this.cloth = cloth;

  const geometry = new BufferGeometry();
  let vertices = cloth.vertices();
  geometry.setAttribute("position", new BufferAttribute(vertices, 3));

  let faces = cloth.faces();
  geometry.setIndex(new BufferAttribute(faces, 1));
  geometry.computeVertexNormals();

  const material = new MeshPhongMaterial({
    color: 0x2194ce,
    side: DoubleSide,
    flatShading: true,
  });
  const mesh = new Mesh(geometry, material);

  mesh.frustumCulled = false;
  this.meshes.set("cloth", mesh);
  this.graphics.scene.add(mesh);
};

Simulator.prototype.updateCloth = function () {
  let mesh = this.meshes.get("cloth");
  let vertices = this.cloth.vertices();
  let position = mesh.geometry.getAttribute("position");
  position.array.set(vertices);
  position.needsUpdate = true;
};
