import {
  BoxGeometry,
  DoubleSide,
  EdgesGeometry,
  LineBasicMaterial,
  LineSegments,
  Matrix4,
  Mesh,
  MeshPhongMaterial,
} from "three";
import { Simulator } from "./Simulator";
import { FloatArrayType } from "./type";

declare module "./Simulator" {
  interface Simulator {
    addCube(length: number): void;
    updateCube(poses: FloatArrayType): void;
  }
}

Simulator.prototype.addCube = function (length: number) {
  const geometry = new BoxGeometry(length, length, length);
  const material = new MeshPhongMaterial({
    color: 0x00ff00,
    side: DoubleSide, // Render both sides of faces
    flatShading: true,
  });
  const cube = new Mesh(geometry, material);

  let name = "cube";
  this.meshes.set(name, cube);
  this.graphics.scene.add(cube);

  const edges = new EdgesGeometry(geometry);
  const edgesMaterial = new LineBasicMaterial({ color: 0x000000 });
  const edgesMesh = new LineSegments(edges, edgesMaterial);
  this.edgesMeshes.set(name, edgesMesh);
  this.graphics.scene.add(edgesMesh);
};

Simulator.prototype.updateCube = function (poses: FloatArrayType) {
  if (poses.length != 7) {
    throw new Error(`cube poses len != 7`);
  }

  this.setPose("cube", poses);
};
