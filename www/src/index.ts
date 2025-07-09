import { createSO101 } from "gorilla-physics";
import { Simulator } from "./Simulator";

export const keysPressed: Record<string, boolean> = {};

document.addEventListener("keydown", (e) => {
  keysPressed[e.key.toLowerCase()] = true;
});

document.addEventListener("keyup", (e) => {
  keysPressed[e.key.toLowerCase()] = false;
});

import("gorilla-physics").then((gorilla) => {
  let default_z = 0.8;

  let angle: number = (0.0 * Math.PI) / 180.0;
  let normal = new Float32Array([Math.sin(angle), 0.0, Math.cos(angle)]);
  let h_ground = 0.0;
  let alpha = 1.0;
  let mu = 1.0;

  let dt = 1.0 / (60.0 * 50.0);

  let l_cube = 1.0;
  // let state = gorilla.createGripper();
  // let state = gorilla.createPusher();
  // let state = gorilla.createCube(l_cube);
  // let delta_z = 0.8;
  // let state = gorilla.createQuadruped(1.0, delta_z);
  // state.addHalfSpace(normal, h_ground);

  // let controller = gorilla.createNullController();
  // let controller = gorilla.createPusherController();
  // let controller = gorilla.createQuadrupedTrottingController(dt, 0.0, -delta_z);
  // let interfaceSimulator = new gorilla.InterfaceSimulator(state, controller);

  // let simulator = new Simulator(interfaceSimulator);
  // let simulator = new Simulator(null);

  // createFEMBox().then((box) => {
  //   simulator.addDeformable(box);
  // });

  // simulator.addPusher();
  // simulator.addCube(l_cube);
  // simulator.addQuadruped();
  // simulator.addPlane(normal, h_ground, 100);

  createSO101().then((state) => {
    state.addHalfSpace(normal, h_ground);

    // let controller = gorilla.createNullController();
    let controller = gorilla.createSO101PositionController();
    let interfaceSimulator = new gorilla.InterfaceSimulator(state, controller);
    let simulator = new Simulator(interfaceSimulator);

    simulator.addMesh(0, "mesh0");
    simulator.addMesh(1, "mesh1");
    simulator.addMesh(2, "mesh2");
    simulator.addMesh(3, "mesh3");
    simulator.addMesh(4, "mesh4");
    simulator.addMesh(5, "gripper");
    simulator.addPlane(normal, h_ground, 100);

    // simulator.updateMesh(0, "mesh0");
    // simulator.updateMesh(1, "mesh1");
    // simulator.updateMesh(2, "mesh2");
    // simulator.updateMesh(3, "mesh3");
    // simulator.updateMesh(4, "mesh4");
    // simulator.updateMesh(5, "gripper");

    // Important: Set initial camera position
    let cameraPosition = {
      eye: { x: 0.0, y: -0.5, z: 0.5 },
      target: { x: 0.0, y: 0, z: 0.0 },
    };
    simulator.graphics.lookAt(cameraPosition);

    simulator.run();
  });
});
