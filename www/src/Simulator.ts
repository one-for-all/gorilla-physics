import { InterfaceSimulator } from "gorilla-physics";
import * as THREE from "three";
import { Graphics } from "./Graphics";

export class Simulator {
  simulator: InterfaceSimulator;

  graphics: Graphics;
  length: number;
  rod: THREE.Mesh;
  rod2: THREE.Mesh;
  bob: THREE.Mesh;
  cart: THREE.Mesh;
  meshes: Map<string, THREE.Mesh>;

  fps: number;
  time: number;

  constructor(simulator: InterfaceSimulator) {
    this.simulator = simulator;

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
    const normal = new THREE.Vector3().fromArray(normalArray);
    const position = normal.multiplyScalar(distance);

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

  addSphere(radius: number) {
    const sphereGeometry = new THREE.SphereGeometry(radius, 32, 32);
    const sphereMaterial = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
    const sphere = new THREE.Mesh(sphereGeometry, sphereMaterial);
    this.meshes.set("sphere", sphere);
    this.graphics.scene.add(sphere);
  }

  addHopper(w_body: number, h_body: number, r_leg: number, r_foot: number) {
    const bodyGeometry = new THREE.BoxGeometry(w_body, w_body, h_body);
    const bodyMaterial = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
    const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
    this.meshes.set("hopper_body", body);
    this.graphics.scene.add(body);

    const legGeometry = new THREE.SphereGeometry(r_leg, 32, 32);
    const legMaterial = new THREE.MeshBasicMaterial({ color: 0xffffff });
    const leg = new THREE.Mesh(legGeometry, legMaterial);
    this.meshes.set("hopper_leg", leg);
    this.graphics.scene.add(leg);

    const footGeometry = new THREE.SphereGeometry(r_foot, 32, 32);
    const footMaterial = new THREE.MeshBasicMaterial({ color: 0x0000ff });
    const foot = new THREE.Mesh(footGeometry, footMaterial);
    this.meshes.set("hopper_foot", foot);
    this.graphics.scene.add(foot);
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

  updateSpherePose(x: number, y: number, z: number) {
    let sphere = this.meshes.get("sphere");
    sphere.position.set(x, y, z);
  }

  updateHopper(poses: Float32Array) {
    let body_euler = [poses[0], poses[1], poses[2]];
    let body_pos = [poses[3], poses[4], poses[5]];

    let body = this.meshes.get("hopper_body");
    body.rotation.set(body_euler[0], body_euler[1], body_euler[2]);
    body.position.set(body_pos[0], body_pos[1], body_pos[2]);

    let leg_euler = [poses[6], poses[7], poses[8]];
    let leg_pos = [poses[9], poses[10], poses[11]];

    let leg = this.meshes.get("hopper_leg");
    leg.rotation.set(leg_euler[0], leg_euler[1], leg_euler[2]);
    leg.position.set(leg_pos[0], leg_pos[1], leg_pos[2]);

    let foot_euler = [poses[12], poses[13], poses[14]];
    let foot_pos = [poses[15], poses[16], poses[17]];

    let foot = this.meshes.get("hopper_foot");
    foot.rotation.set(foot_euler[0], foot_euler[1], foot_euler[2]);
    foot.position.set(foot_pos[0], foot_pos[1], foot_pos[2]);
  }

  run(timestamp?: number) {
    this.graphics.render();

    // TODO: measure and use the actual time elapsed
    const dt = 1 / this.fps;

    // this.updateCartPose(pos);
    // this.updateCartPole(qs[0], qs[1]);
    let qs = this.simulator.step(dt);
    this.time += dt;

    let poses = this.simulator.poses();
    this.updateHopper(poses);

    requestAnimationFrame((t) => this.run(t));
  }
}
