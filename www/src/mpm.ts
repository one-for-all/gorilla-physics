import { InterfaceMPMDeformable } from "gorilla-physics";
import {
  BufferAttribute,
  BufferGeometry,
  DoubleSide,
  DynamicDrawUsage,
  InstancedMesh,
  Mesh,
  MeshPhongMaterial,
  MeshPhysicalMaterial,
  Object3D,
  SphereGeometry,
} from "three";
import { Simulator } from "./Simulator";

declare module "./Simulator" {
  interface Simulator {
    addMPMDeformable(cloth: InterfaceMPMDeformable): void;
    updateMPMDeformable(): void;
  }
}

Simulator.prototype.addMPMDeformable = function (
  deformable: InterfaceMPMDeformable,
) {
  this.mpmDeformable = deformable;

  let radius = 1e-2;
  const geometry = new SphereGeometry(radius, 8, 8);
  const material = new MeshPhysicalMaterial({
    color: 0x3366ff,
    roughness: 0,
    transmission: 1.0,
    ior: 1.33, // index of refraction for water
    transparent: true,
    opacity: 0.7,
  });

  let particles = deformable.particles();
  let count = particles.length / 2;
  const mesh = new InstancedMesh(geometry, material, count);
  mesh.frustumCulled = false;
  mesh.instanceMatrix.setUsage(DynamicDrawUsage);
  this.mpm2DMesh = mesh;
  this.graphics.scene.add(mesh);

  const dummy = new Object3D();
  for (let i = 0; i < particles.length; i += 2) {
    let x = particles[i];
    let y = 0.0;
    let z = particles[i + 1];
    dummy.position.set(x, y, z);
    dummy.updateMatrix();
    mesh.setMatrixAt(i / 2, dummy.matrix);
  }

  mesh.instanceMatrix.needsUpdate = true;
};

Simulator.prototype.updateMPMDeformable = function () {
  let particles = this.mpmDeformable.particles();
  const dummy = new Object3D();
  for (let i = 0; i < particles.length; i += 2) {
    let x = particles[i];
    let y = 0.0;
    let z = particles[i + 1];
    dummy.position.set(x, y, z);
    dummy.updateMatrix();
    this.mpm2DMesh.setMatrixAt(i / 2, dummy.matrix);
  }
  this.mpm2DMesh.instanceMatrix.needsUpdate = true;
};
