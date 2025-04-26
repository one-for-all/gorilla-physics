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
  let state = gorilla.createGripper();

  let angle: number = (0.0 * Math.PI) / 180.0;
  let normal = new Float32Array([Math.sin(angle), 0.0, Math.cos(angle)]);
  let h_ground = 0.0;
  let alpha = 1.0;
  let mu = 1.0;
  state.addHalfSpaceCustom(normal, h_ground, alpha, mu);

  let dt = 1.0 / (60.0 * 50.0);
  let controller = gorilla.createGripperontroller();

  let interfaceSimulator = new gorilla.InterfaceSimulator(state, controller);

  let simulator = new Simulator(interfaceSimulator);
  simulator.addGripper();

  simulator.addPlane(normal, h_ground, 100);

  // Important: Set initial camera position
  let cameraPosition = {
    eye: { x: 0.0, y: -10.0, z: 1.0 },
    target: { x: 0.0, y: 0, z: 0.0 },
  };
  simulator.graphics.lookAt(cameraPosition);

  simulator.run();
});
