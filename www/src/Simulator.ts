import {
  InterfaceMassSpringDeformable,
  InterfaceSimulator,
} from "gorilla-physics";
import * as THREE from "three";
import { Graphics } from "./Graphics";

import { Line2 } from "three/examples/jsm/lines/Line2";
import { LineGeometry } from "three/examples/jsm/lines/LineGeometry";
import { LineMaterial } from "three/examples/jsm/lines/LineMaterial";

export class Simulator {
  simulator: InterfaceSimulator;

  massSpringBunny: InterfaceMassSpringDeformable;

  graphics: Graphics;
  length: number;
  rod: THREE.Mesh;
  rod2: THREE.Mesh;
  bob: THREE.Mesh;
  cart: THREE.Mesh;
  meshes: Map<string, THREE.Mesh>;
  edgesMeshes: Map<string, THREE.LineSegments>;
  lines: Map<string, Line2>;

  fps: number;
  time: number;

  constructor(simulator: InterfaceSimulator) {
    this.simulator = simulator;

    this.massSpringBunny = null;

    this.graphics = new Graphics();
    this.meshes = new Map();
    this.edgesMeshes = new Map();
    this.lines = new Map();
    this.fps = 60;
    this.time = 0.0;
  }

  addBunny(bunny: InterfaceMassSpringDeformable) {
    this.massSpringBunny = bunny;

    const geometry = new THREE.BufferGeometry();
    let vertices = this.massSpringBunny.vertices();
    geometry.setAttribute("position", new THREE.BufferAttribute(vertices, 3));

    let facets = this.massSpringBunny.facets();
    geometry.setIndex(new THREE.BufferAttribute(facets, 1));
    geometry.computeVertexNormals();

    const material = new THREE.MeshPhongMaterial({
      color: 0x2194ce,
      side: THREE.DoubleSide, // Render both sides of faces
      flatShading: true,
    });
    const mesh = new THREE.Mesh(geometry, material);
    this.meshes.set("bunny", mesh);
    this.graphics.scene.add(mesh);
  }

  updateBunny() {
    let bunny = this.meshes.get("bunny");

    let vertices = this.massSpringBunny.vertices();
    bunny.geometry.setAttribute(
      "position",
      new THREE.BufferAttribute(vertices, 3)
    );
  }

  addBox(name: string, color: number, w: number, d: number, h: number) {
    const geometry = new THREE.BoxGeometry(w, d, h);
    const material = new THREE.MeshBasicMaterial({ color: color });
    const box = new THREE.Mesh(geometry, material);
    this.meshes.set(name, box);
    this.graphics.scene.add(box);

    const edges = new THREE.EdgesGeometry(geometry);
    const edgesMaterial = new THREE.LineBasicMaterial({ color: 0x000000 });
    const edgesMesh = new THREE.LineSegments(edges, edgesMaterial);
    this.edgesMeshes.set(name, edgesMesh);
    this.graphics.scene.add(edgesMesh);
  }

  addSphere(name: string, color: number, radius: number) {
    const geometry = new THREE.SphereGeometry(radius, 32, 32);
    const material = new THREE.MeshBasicMaterial({ color: color });
    const sphere = new THREE.Mesh(geometry, material);
    this.meshes.set(name, sphere);
    this.graphics.scene.add(sphere);
  }

  addLine(name: string, color: number, linewidth: number) {
    const geometry = new LineGeometry();
    const material = new LineMaterial({
      color: color,
      linewidth: linewidth, // Adjust thickness
      resolution: new THREE.Vector2(window.innerWidth, window.innerHeight), // Required for LineMaterial
    });
    const line = new Line2(geometry, material);
    this.lines.set(name, line);
    this.graphics.scene.add(line);
  }

  setPose(name: string, poses: Float32Array) {
    let euler = [poses[0], poses[1], poses[2]];
    let pos = [poses[3], poses[4], poses[5]];

    let body = this.meshes.get(name);
    body.rotation.set(euler[0], euler[1], euler[2]);
    body.position.set(pos[0], pos[1], pos[2]);

    let edges = this.edgesMeshes.get(name);
    if (edges) {
      edges.rotation.set(euler[0], euler[1], euler[2]);
      edges.position.set(pos[0], pos[1], pos[2]);
    }
  }

  setLineEndpoints(name: string, pos1: Float32Array, pos2: Float32Array) {
    this.lines
      .get(name)
      .geometry.setPositions([
        pos1[0],
        pos1[1],
        pos1[2],
        pos2[0],
        pos2[1],
        pos2[2],
      ]);
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

  addCompassGait() {
    this.addSphere("hip", 0xff0000, 0.3);
    this.addBox("left leg", 0x00ff00, 0.1, 0.1, 2.0);
    this.addBox("right leg", 0x00ff00, 0.1, 0.1, 2.0);
  }

  addAcuatedAnkleHopper() {
    this.addSphere("foot", 0xffffff, 0.1);
    this.addSphere("ankle", 0x000000, 0.05);
    this.addSphere("body", 0x00ff00, 0.1);
  }

  addAcuatedHipHopper() {
    this.addBox("foot", 0xffffff, 0.2, 0.2, 0.2);
    this.addSphere("hip", 0x000000, 0.05);
    this.addBox("body", 0x00ff00, 0.2, 0.2, 0.2);
  }

  addQuadruped() {
    let l_hip = 0.2;
    let l_knee = 0.2;
    this.addBox("body", 0xffffff, 1.5, 0.5, 0.5);
    this.addBox("fr_hip", 0xff0000, l_hip, l_hip, l_hip);
    this.addBox("fr_knee", 0x00ff00, l_knee, l_knee, l_knee);
    this.addBox("fl_hip", 0xff0000, l_hip, l_hip, l_hip);
    this.addBox("fl_knee", 0x00ff00, l_knee, l_knee, l_knee);
    this.addBox("br_hip", 0xff0000, l_hip, l_hip, l_hip);
    this.addBox("br_knee", 0x00ff00, l_knee, l_knee, l_knee);
    this.addBox("bl_hip", 0xff0000, l_hip, l_hip, l_hip);
    this.addBox("bl_knee", 0x00ff00, l_knee, l_knee, l_knee);

    let radius_foot = 0.1;
    for (let i = 0; i < 12; i++) {
      const footGeometry = new THREE.SphereGeometry(radius_foot, 32, 32);
      const footMaterial = new THREE.MeshBasicMaterial({ color: 0x0000ff });
      const foot = new THREE.Mesh(footGeometry, footMaterial);
      this.meshes.set(`foot ${i + 1}`, foot);
      this.graphics.scene.add(foot);
    }
  }

  addPusher() {
    this.addBox("extension", 0xffffff, 0.5, 0.5, 0.5);
    this.addBox("lift", 0xff0000, 0.2, 0.2, 1.75);
    this.addBox("cube", 0x00ff00, 0.5, 0.5, 0.5);
  }

  addGripper() {
    this.addBox("lift", 0xff0000, 0.2, 0.2, 1.0);
    this.addBox("gripper_left", 0x00ff00, 0.1, 0.4, 0.4);
    this.addBox("gripper_right", 0x00ff00, 0.1, 0.4, 0.4);
    this.addBox("cube", 0x0000ff, 0.5, 0.5, 0.5);
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
    r_piston: number,
    l_leg: number
  ) {
    this.addBox("hopper_body", 0x00ff00, w_body, w_body, h_body);
    this.addSphere("hopper_hip", 0xff0000, r_hip);
    this.addLine("hopper_body_hip_line", 0xff0000, 3);
    this.addSphere("hopper_piston", 0xffffff, r_piston);
    this.addBox("hopper_leg", 0x0000ff, 0.1, 0.1, l_leg);
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

  updateCompassGait(poses: Float32Array) {
    let hipPose = poses.subarray(0, 6);
    this.setPose("hip", hipPose);

    let leftLegPose = poses.subarray(6, 12);
    this.setPose("left leg", leftLegPose);

    let rightLegPose = poses.subarray(12, 18);
    this.setPose("right leg", rightLegPose);
  }

  updateActuatedAnkleHopper(poses: Float32Array) {
    let footPose = poses.subarray(0, 6);
    this.setPose("foot", footPose);

    let anklePose = poses.subarray(6, 12);
    this.setPose("ankle", anklePose);

    let bodyPose = poses.subarray(12, 18);
    this.setPose("body", bodyPose);
  }

  updateActuatedHipHopper(poses: Float32Array) {
    let footPose = poses.subarray(0, 6);
    this.setPose("foot", footPose);

    let hipPose = poses.subarray(6, 12);
    this.setPose("hip", hipPose);

    let bodyPose = poses.subarray(12, 18);
    this.setPose("body", bodyPose);
  }

  updateQuadruped(poses: Float32Array, contact_positions: Float32Array) {
    let i = 0;
    let bodyPose = poses.subarray(i, i + 6);
    this.setPose("body", bodyPose);

    i += 6;
    let frHipPose = poses.subarray(i, i + 6);
    this.setPose("fr_hip", frHipPose);

    i += 6;
    let frKneePose = poses.subarray(i, i + 6);
    this.setPose("fr_knee", frKneePose);

    i += 6;
    let flHipPose = poses.subarray(i, i + 6);
    this.setPose("fl_hip", flHipPose);

    i += 6;
    let flKneePose = poses.subarray(i, i + 6);
    this.setPose("fl_knee", flKneePose);

    i += 6;
    let brHipPose = poses.subarray(i, i + 6);
    this.setPose("br_hip", brHipPose);

    i += 6;
    let brKneePose = poses.subarray(i, i + 6);
    this.setPose("br_knee", brKneePose);

    i += 6;
    let blHipPose = poses.subarray(i, i + 6);
    this.setPose("bl_hip", blHipPose);

    i += 6;
    let blKneePose = poses.subarray(i, i + 6);
    this.setPose("bl_knee", blKneePose);

    for (let i = 0; i < contact_positions.length; i += 3) {
      let foot = this.meshes.get(`foot ${i / 3 + 1}`);
      foot.position.set(
        contact_positions[i],
        contact_positions[i + 1],
        contact_positions[i + 2]
      );
    }
  }

  updatePusher(poses: Float32Array) {
    let i = 0;
    let extensionPose = poses.subarray(i, i + 6);
    this.setPose("extension", extensionPose);

    i += 6;
    let liftPose = poses.subarray(i, i + 6);
    this.setPose("lift", liftPose);

    i += 6;
    let cubePose = poses.subarray(i, i + 6);
    this.setPose("cube", cubePose);
  }

  // TODO: automate this
  updateGripper(poses: Float32Array) {
    let i = 0;
    let liftPose = poses.subarray(i, i + 6);
    this.setPose("lift", liftPose);

    i += 6;
    let gripperLeftPose = poses.subarray(i, i + 6);
    this.setPose("gripper_left", gripperLeftPose);

    i += 6;
    let gripperRightPose = poses.subarray(i, i + 6);
    this.setPose("gripper_right", gripperRightPose);

    i += 6;
    let cubePose = poses.subarray(i, i + 6);
    this.setPose("cube", cubePose);
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
    let bodyPose = poses.subarray(0, 6);
    this.setPose("hopper_body", bodyPose);

    let hipPose = poses.subarray(6, 12);
    this.setPose("hopper_hip", hipPose);

    this.setLineEndpoints(
      "hopper_body_hip_line",
      bodyPose.subarray(3, 6),
      hipPose.subarray(3, 6)
    );

    this.setPose("hopper_piston", poses.subarray(12, 18));

    let legGeometry = this.meshes.get("hopper_leg")
      .geometry as THREE.BoxGeometry;
    let legLength = legGeometry.parameters.depth;
    let legPose = poses.subarray(18, 24);

    let legEuler = new THREE.Euler(legPose[0], legPose[1], legPose[2]);
    let legQuaternion = new THREE.Quaternion();
    legQuaternion.setFromEuler(legEuler);

    let legPositionOffset = new THREE.Vector3(0, 0, legLength / 2.0);
    legPositionOffset.applyQuaternion(legQuaternion);

    legPose[3] += legPositionOffset.x;
    legPose[4] += legPositionOffset.y;
    legPose[5] += legPositionOffset.z;

    this.setPose("hopper_leg", legPose);
  }

  run(timestamp?: number) {
    this.graphics.render();

    // TODO: measure and use the actual time elapsed
    const dt = 1 / this.fps;

    // let control_input = new Float32Array(2);

    // // drop the arm
    // if (keysPressed["s"]) {
    //   control_input[0] = 1.0;
    // } else {
    //   control_input[0] = 0.0;
    // }

    // // close the gripper
    // if (keysPressed[" "]) {
    //   control_input[1] = 1.0;
    // } else {
    //   control_input[1] = 0.0;
    // }

    // let qs = this.simulator.step(dt, control_input);
    if (this.massSpringBunny) {
      // drag the body towards right
      let tau = new Float32Array(this.massSpringBunny.vertices().length);
      tau[499 * 3] = 100.0;

      this.massSpringBunny.step(dt, tau);
      this.updateBunny();
    }
    this.time += dt;

    // let poses = this.simulator.poses();
    // let contact_positions = this.simulator.contact_positions();
    // this.updateGripper(poses);

    requestAnimationFrame((t) => this.run(t));
  }
}
