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
};

Simulator.prototype.updateBiped = function (poses: FloatArrayType) {
  if (poses.length != 5 * 7) {
    throw new Error("poses len != 3 * 7");
  }

  let frames = ["base", "pelvis_left", "hip_left", "thigh_left", "calf_left"];

  for (let i = 0; i < poses.length; i += 7) {
    let pose = poses.subarray(i, i + 7);
    this.setPose(frames[i / 7], pose);
  }
};
