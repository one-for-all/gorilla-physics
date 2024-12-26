import { InterfaceMechanismState } from "gorilla-physics";
import * as THREE from "three";
import { Graphics } from "./Graphics";

export class Simulator {
  mechanismState: InterfaceMechanismState;

  graphics: Graphics;
  length: number;
  rod: THREE.Mesh;
  bob: THREE.Mesh;

  fps: number;

  constructor(mechanismState: InterfaceMechanismState) {
    this.mechanismState = mechanismState;

    this.graphics = new Graphics();
    this.fps = 60;
  }

  addRodPendulum(length: number) {
    this.length = length;

    const rodGeometry = new THREE.CylinderGeometry(0.1, 0.1, length, 32);
    const rodMaterial = new THREE.MeshBasicMaterial({ color: 0xffffff });
    this.rod = new THREE.Mesh(rodGeometry, rodMaterial);
    this.rod.position.y = -length / 2;
    this.graphics.scene.add(this.rod);

    const bobGeometry = new THREE.SphereGeometry(0.2, 32, 32);
    const bobMaterial = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
    this.bob = new THREE.Mesh(bobGeometry, bobMaterial);
    this.graphics.scene.add(this.bob);
  }

  updateRodPose(angle: number) {
    // Update rod center-of-shape position
    this.rod.position.x = (this.length / 2) * Math.sin(angle);
    this.rod.position.y = (-this.length / 2) * Math.cos(angle);

    // Rotate rod
    this.rod.rotation.z = angle;
  }

  run(timestamp?: number) {
    this.graphics.render();

    // TODO: measure and use the actual time elapsed
    const dt = 1 / this.fps;

    let angle = -(this.mechanismState.step(dt)[0] - Math.PI / 2); // Coordinate transform

    this.updateRodPose(angle);

    requestAnimationFrame((t) => this.run(t));
  }
}
