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
};

Simulator.prototype.updateBiped = function (poses: FloatArrayType) {
  let i = 0;
  let base_pose = poses.subarray(i, i + 6);
  this.setPose("base", base_pose);

  i += 6;
  let pelvis_left_pose = poses.subarray(i, i + 6);
  this.setPose("pelvis_left", pelvis_left_pose);

  i += 6;
  let hip_left_pose = poses.subarray(i, i + 6);
  this.setPose("hip_left", hip_left_pose);
};
