import { InterfaceMechanismState } from "gorilla-physics";
import * as THREE from "three";
import { Graphics } from "./Graphics";

export class Simulator {
  mechanismState: InterfaceMechanismState;

  graphics: Graphics;
  length: number;
  rod: THREE.Mesh;
  rod2: THREE.Mesh;
  bob: THREE.Mesh;
  cart: THREE.Mesh;
  meshes: Map<string, THREE.Mesh>;

  fps: number;
  time: number;

  constructor(mechanismState: InterfaceMechanismState) {
    this.mechanismState = mechanismState;

    this.graphics = new Graphics();
    this.meshes = new Map();
    this.fps = 60;
    this.time = 0.0;
  }

  addCart(length: number) {
    this.length = length;
    const cartGeometry = new THREE.BoxGeometry(length, 1, 1);
    const cartMaterial = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
    this.cart = new THREE.Mesh(cartGeometry, cartMaterial);
    this.graphics.scene.add(this.cart);
  }

  addCartPole(length: number) {
    const cartGeometry = new THREE.BoxGeometry(3, 0.1, 0.1);
    const cartMaterial = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
    const cart = new THREE.Mesh(cartGeometry, cartMaterial);
    this.meshes.set("cart", cart);
    this.graphics.scene.add(cart);

    const poleGeometry = new THREE.CylinderGeometry(0.1, 0.1, length, 32);
    const poleMaterial = new THREE.MeshBasicMaterial({ color: 0xffffff });
    const pole = new THREE.Mesh(poleGeometry, poleMaterial);
    pole.position.y = -length / 2;
    this.meshes.set("pole", pole);
    this.graphics.scene.add(pole);
  }

  addDoublemPendulum(length: number) {
    this.length = length;

    const bobGeometry = new THREE.SphereGeometry(0.2, 32, 32);
    const bobMaterial = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
    this.bob = new THREE.Mesh(bobGeometry, bobMaterial);
    this.graphics.scene.add(this.bob);

    const rodGeometry = new THREE.CylinderGeometry(0.1, 0.1, length, 32);
    const rodMaterial = new THREE.MeshBasicMaterial({ color: 0xffffff });
    this.rod = new THREE.Mesh(rodGeometry, rodMaterial);
    this.rod.position.y = -length / 2;
    this.graphics.scene.add(this.rod);

    const rod2Geometry = new THREE.CylinderGeometry(0.1, 0.1, length, 32);
    const rod2Material = new THREE.MeshBasicMaterial({ color: 0xffffff });
    this.rod2 = new THREE.Mesh(rod2Geometry, rod2Material);
    this.rod2.position.y = -length / 2 - length;
    this.graphics.scene.add(this.rod2);
  }

  addPendulum(length: number) {
    const pivotGeometry = new THREE.SphereGeometry(0.2, 32, 32);
    const pivotMaterial = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
    const pivot = new THREE.Mesh(pivotGeometry, pivotMaterial);
    this.meshes.set("pivot", pivot);
    this.graphics.scene.add(pivot);

    const rodGeometry = new THREE.CylinderGeometry(0.1, 0.1, length, 32);
    const rodMaterial = new THREE.MeshBasicMaterial({ color: 0xffffff });
    const rod = new THREE.Mesh(rodGeometry, rodMaterial);
    this.meshes.set("rod", rod);
    this.graphics.scene.add(rod);
  }

  addPlane(normalArray: Float32Array, distance: number, size: number) {
    const quaternion = new THREE.Quaternion();
    quaternion.setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI / 2);

    const normal = new THREE.Vector3()
      .fromArray(normalArray)
      .applyQuaternion(quaternion);
    const position = normal.clone().multiplyScalar(distance);

    const planeGeometry = new THREE.PlaneGeometry(size, size);
    const planeMaterial = new THREE.MeshPhongMaterial({
      color: 0x808080,
      side: THREE.DoubleSide, // Ensure it can be viewed from both sides
    });

    const plane = new THREE.Mesh(planeGeometry, planeMaterial);
    plane.position.copy(position);
    plane.lookAt(position.clone().add(normal));
    this.graphics.scene.add(plane);
  }

  addRodPendulum(length: number) {
    this.length = length;

    const bobGeometry = new THREE.SphereGeometry(0.2, 32, 32);
    const bobMaterial = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
    this.bob = new THREE.Mesh(bobGeometry, bobMaterial);
    this.graphics.scene.add(this.bob);

    const rodGeometry = new THREE.CylinderGeometry(0.1, 0.1, length, 32);
    const rodMaterial = new THREE.MeshBasicMaterial({ color: 0xffffff });
    this.rod = new THREE.Mesh(rodGeometry, rodMaterial);
    this.rod.position.y = -length / 2;
    this.graphics.scene.add(this.rod);
  }

  updateRodPose(angle: number) {
    // Update rod center-of-shape position
    this.rod.position.x = (this.length / 2) * Math.sin(angle);
    this.rod.position.y = (-this.length / 2) * Math.cos(angle);

    // Rotate rod
    this.rod.rotation.z = angle;
  }

  updateDoublemPendulumPose(angle1: number, angle2: number) {
    this.updateRodPose(angle1);

    this.rod2.position.x =
      this.length * Math.sin(angle1) +
      (this.length / 2) * Math.sin(angle1 + angle2);
    this.rod2.position.y =
      -this.length * Math.cos(angle1) -
      (this.length / 2) * Math.cos(angle1 + angle2);
    this.rod2.rotation.z = angle1 + angle2;
  }

  updateCartPose(pos: number) {
    this.cart.position.x = pos;
  }

  updateCartPole(cart_pos: number, pole_angle: number) {
    let cart = this.meshes.get("cart");
    cart.position.x = cart_pos;

    let pole = this.meshes.get("pole");
    const length = (pole.geometry as THREE.CylinderGeometry).parameters.height;
    pole.position.x = cart_pos + (length / 2) * Math.sin(pole_angle);
    pole.position.y = -(length / 2) * Math.cos(pole_angle);
    pole.rotation.z = pole_angle;
  }

  updatePendulumPose(angle: number) {
    let rod = this.meshes.get("rod");
    const length = (rod.geometry as THREE.CylinderGeometry).parameters.height;
    rod.position.x = (length / 2) * Math.sin(angle);
    rod.position.y = (-length / 2) * Math.cos(angle);
    rod.rotation.z = angle;
  }

  run(timestamp?: number) {
    this.graphics.render();

    // TODO: measure and use the actual time elapsed
    const dt = 1 / this.fps;

    let qs = this.mechanismState.step(dt);
    // this.updateCartPose(pos);
    // this.updateCartPole(qs[0], qs[1]);
    this.time += dt;
    // console.log("t :", this.time.toFixed(2));

    // Coordinate transform from mechanism to graphics
    let angle = -qs[0] + Math.PI / 2;
    this.updatePendulumPose(angle);
    // let angle1 = qs[0] + Math.PI / 2;
    // let angle2 = qs[1];
    // this.updateDoublemPendulumPose(angle1, angle2);

    requestAnimationFrame((t) => this.run(t));
  }
}
