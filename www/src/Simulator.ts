import { InterfaceSimulator } from "gorilla-physics";
import * as THREE from "three";
import { Graphics } from "./Graphics";

import { Line2 } from "three/examples/jsm/lines/Line2";
import { LineGeometry } from "three/examples/jsm/lines/LineGeometry";
import { LineMaterial } from "three/examples/jsm/lines/LineMaterial";

export class Simulator {
  simulator: InterfaceSimulator;

  graphics: Graphics;
  length: number;
  rod: THREE.Mesh;
  rod2: THREE.Mesh;
  bob: THREE.Mesh;
  cart: THREE.Mesh;
  meshes: Map<string, THREE.Mesh>;
  lines: Map<string, Line2>;

  fps: number;
  time: number;

  constructor(simulator: InterfaceSimulator) {
    this.simulator = simulator;

    this.graphics = new Graphics();
    this.meshes = new Map();
    this.lines = new Map();
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

  addCube(length: number) {
    const cubeGeometry = new THREE.BoxGeometry(length, length, length);
    const cubeMaterial = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
    const cube = new THREE.Mesh(cubeGeometry, cubeMaterial);
    this.meshes.set("cube", cube);
    this.graphics.scene.add(cube);
  }

  addRimlessWheel(radius: number, length: number, n_foot: number) {
    // Add pivot at the center
    const pivotGeometry = new THREE.SphereGeometry(radius, 32, 32);
    const pivotMaterial = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
    const pivot = new THREE.Mesh(pivotGeometry, pivotMaterial);
    this.meshes.set("pivot", pivot);
    this.graphics.scene.add(pivot);

    // Add feet at the peripheral
    let radius_foot = 1.0;
    for (let i = 0; i < n_foot; i++) {
      const footGeometry = new THREE.SphereGeometry(radius_foot, 32, 32);
      const footMaterial = new THREE.MeshBasicMaterial({ color: 0xffffff });
      const foot = new THREE.Mesh(footGeometry, footMaterial);
      this.meshes.set(`foot ${i + 1}`, foot);
      this.graphics.scene.add(foot);
    }
  }

  add1DHopper(w_body: number, h_body: number, r_leg: number, r_foot: number) {
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

  add2DHopper(
    w_body: number,
    h_body: number,
    r_hip: number,
    r_leg: number,
    r_foot: number
  ) {
    const bodyGeometry = new THREE.BoxGeometry(w_body, w_body, h_body);
    const bodyMaterial = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
    const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
    this.meshes.set("hopper_body", body);
    this.graphics.scene.add(body);

    const hipGeometry = new THREE.SphereGeometry(r_hip, 32, 32);
    const hipMaterial = new THREE.MeshBasicMaterial({ color: 0xff0000 });
    const hip = new THREE.Mesh(hipGeometry, hipMaterial);
    this.meshes.set("hopper_hip", hip);
    this.graphics.scene.add(hip);

    const hipLineGeometry = new LineGeometry();
    const hipLineMaterial = new LineMaterial({
      color: 0xff0000,
      linewidth: 3, // Adjust thickness
      resolution: new THREE.Vector2(window.innerWidth, window.innerHeight), // Required for LineMaterial
    });
    const hipLine = new Line2(hipLineGeometry, hipLineMaterial);
    this.lines.set("hopper_hip_line", hipLine);
    this.graphics.scene.add(hipLine);

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

  updateSpherePose(poses: Float32Array) {
    let euler = [poses[0], poses[1], poses[2]];
    let pos = [poses[3], poses[4], poses[5]];

    let body = this.meshes.get("sphere");
    body.rotation.set(euler[0], euler[1], euler[2]);
    body.position.set(pos[0], pos[1], pos[2]);
  }

  updateCube(poses: Float32Array) {
    let euler = [poses[0], poses[1], poses[2]];
    let pos = [poses[3], poses[4], poses[5]];

    let body = this.meshes.get("cube");
    body.rotation.set(euler[0], euler[1], euler[2]);
    body.position.set(pos[0], pos[1], pos[2]);
  }

  updateRimlessWheel(poses: Float32Array, contact_positions: Float32Array) {
    let euler = [poses[0], poses[1], poses[2]];
    let pos = [poses[3], poses[4], poses[5]];

    let body = this.meshes.get("pivot");
    body.rotation.set(euler[0], euler[1], euler[2]);
    body.position.set(pos[0], pos[1], pos[2]);

    for (let i = 0; i < contact_positions.length; i += 3) {
      let foot = this.meshes.get(`foot ${i / 3 + 1}`);
      foot.position.set(
        contact_positions[i],
        contact_positions[i + 1],
        contact_positions[i + 2]
      );
    }
  }

  update1DHopper(poses: Float32Array) {
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

  update2DHopper(poses: Float32Array) {
    let body_euler = [poses[0], poses[1], poses[2]];
    let body_pos = [poses[3], poses[4], poses[5]];

    let body = this.meshes.get("hopper_body");
    body.rotation.set(body_euler[0], body_euler[1], body_euler[2]);
    body.position.set(body_pos[0], body_pos[1], body_pos[2]);

    let hip_euler = [poses[6], poses[7], poses[8]];
    let hip_pos = [poses[9], poses[10], poses[11]];

    let hip = this.meshes.get("hopper_hip");
    hip.rotation.set(hip_euler[0], hip_euler[1], hip_euler[2]);
    hip.position.set(hip_pos[0], hip_pos[1], hip_pos[2]);

    this.lines
      .get("hopper_hip_line")
      .geometry.setPositions([
        body_pos[0],
        body_pos[1],
        body_pos[2],
        hip_pos[0],
        hip_pos[1],
        hip_pos[2],
      ]);

    let leg_euler = [poses[12], poses[13], poses[14]];
    let leg_pos = [poses[15], poses[16], poses[17]];

    let leg = this.meshes.get("hopper_leg");
    leg.rotation.set(leg_euler[0], leg_euler[1], leg_euler[2]);
    leg.position.set(leg_pos[0], leg_pos[1], leg_pos[2]);

    let foot_euler = [poses[18], poses[19], poses[20]];
    let foot_pos = [poses[21], poses[22], poses[23]];

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
    let contact_positions = this.simulator.contact_positions();
    this.updateRimlessWheel(poses, contact_positions);
    // this.updateCube(poses);
    // this.update1DHopper(poses);
    // this.update2DHopper(poses);

    requestAnimationFrame((t) => this.run(t));
  }
}
