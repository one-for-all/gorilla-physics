import { Matrix4 } from "three";
import { Simulator } from "./Simulator";
import { FloatArrayType } from "./type";

declare module "./Simulator" {
  interface Simulator {
    addBiped(): void;
    updateBiped(poses: FloatArrayType): void;
  }
}

Simulator.prototype.addBiped = function () {
  let l1 = 0.05;
  let l2 = 0.2;

  this.addCuboid("base", 0xff0000, l1, l1, l2);
  let pelvisLeftOffset = new Matrix4().makeTranslation(0, 0, -l2 / 2);
  this.addCuboid("pelvis_left", 0x00ff00, l1, l1, l2, pelvisLeftOffset);
  let hipLeftOffset = new Matrix4().makeTranslation(l2 / 2, 0, 0);
  this.addCuboid("hip_left", 0x0000ff, l2, l1, l1, hipLeftOffset);
  let thighLeftOffset = new Matrix4().makeTranslation(0, 0, -l2 / 2);
  this.addCuboid("thigh_left", 0xff0000, l1, l1, l2, thighLeftOffset);
  let calfLeftOffset = new Matrix4().makeTranslation(0, 0, -l2 / 2);
  this.addCuboid("calf_left", 0x00ff00, l1, l1, l2, calfLeftOffset);
  this.addCuboid("foot_left", 0x0000ff, l1, l2, l1);

  let pelvisRightOffset = new Matrix4().makeTranslation(0, 0, -l2 / 2);
  this.addCuboid("pelvis_right", 0x00ff00, l1, l1, l2, pelvisRightOffset);
  let hipRightOffset = new Matrix4().makeTranslation(-l2 / 2, 0, 0);
  this.addCuboid("hip_right", 0x0000ff, l2, l1, l1, hipRightOffset);
  let thighRightOffset = new Matrix4().makeTranslation(0, 0, -l2 / 2);
  this.addCuboid("thigh_right", 0xff0000, l1, l1, l2, thighRightOffset);
  let calfRightOffset = new Matrix4().makeTranslation(0, 0, -l2 / 2);
  this.addCuboid("calf_right", 0x00ff00, l1, l1, l2, calfRightOffset);
  this.addCuboid("foot_right", 0x0000ff, l1, l2, l1);
};

Simulator.prototype.updateBiped = function (poses: FloatArrayType) {
  let frames = [
    "base",
    "pelvis_left",
    "hip_left",
    "thigh_left",
    "calf_left",
    "foot_left",
    "pelvis_right",
    "hip_right",
    "thigh_right",
    "calf_right",
    "foot_right",
  ];

  let n_bodies = frames.length;
  if (poses.length != n_bodies * 7) {
    throw new Error(`poses len != ${n_bodies} * 7`);
  }

  for (let i = 0; i < poses.length; i += 7) {
    let pose = poses.subarray(i, i + 7);
    this.setPose(frames[i / 7], pose);
  }
};

declare module "./Simulator" {
  interface Simulator {
    addLeg(): void;
    updateLeg(poses: FloatArrayType): void;
  }
}

Simulator.prototype.addLeg = function () {
  let l1 = 0.05;
  let l2 = 0.2;

  let thighLeftOffset = new Matrix4().makeTranslation(0, 0, -l2 / 2);
  this.addCuboid("thigh_left", 0xff0000, l1, l1, l2, thighLeftOffset);
  let calfLeftOffset = new Matrix4().makeTranslation(0, 0, -l2 / 2);
  this.addCuboid("calf_left", 0x00ff00, l1, l1, l2, calfLeftOffset);
  this.addCuboid("foot_left", 0x0000ff, l1, l2, l1);
};

Simulator.prototype.updateLeg = function (poses: FloatArrayType) {
  let frames = ["thigh_left", "calf_left", "foot_left"];

  let n_bodies = frames.length;
  if (poses.length != n_bodies * 7) {
    throw new Error(`poses len != ${n_bodies} * 7`);
  }

  for (let i = 0; i < poses.length; i += 7) {
    let pose = poses.subarray(i, i + 7);
    this.setPose(frames[i / 7], pose);
  }
};

declare module "./Simulator" {
  interface Simulator {
    addLegFromFoot(): void;
    updateLegFromFoot(poses: FloatArrayType): void;
  }
}

Simulator.prototype.addLegFromFoot = function () {
  let l1 = 0.05;
  let l2 = 0.2;

  this.addCuboid("foot_left", 0x0000ff, l1, l2, l1);
  let calfLeftOffset = new Matrix4().makeTranslation(0, 0, l2 / 2);
  this.addCuboid("calf_left", 0x00ff00, l1, l1, l2, calfLeftOffset);
  let thighLeftOffset = new Matrix4().makeTranslation(0, 0, l2 / 2);
  this.addCuboid("thigh_left", 0xff0000, l1, l1, l2, thighLeftOffset);
};

Simulator.prototype.updateLegFromFoot = function (poses: FloatArrayType) {
  let frames = ["foot_left", "calf_left", "thigh_left"];

  let n_bodies = frames.length;
  if (poses.length != n_bodies * 7) {
    throw new Error(`poses len != ${n_bodies} * 7`);
  }

  for (let i = 0; i < poses.length; i += 7) {
    let pose = poses.subarray(i, i + 7);
    this.setPose(frames[i / 7], pose);
  }
};
