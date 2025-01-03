import { Simulator } from "./Simulator";

import("gorilla-physics").then((gorilla) => {
  const length = 7.0;
  let mechanismState = gorilla.createDoublePendulum(length);
  let simulator = new Simulator(mechanismState);
  // Add a rod pendulum
  simulator.addDoublemPendulum(length);

  // Important: Set initial camera position
  let cameraPosition = {
    eye: { x: 0.0, y: 10.0, z: 20.0 },
    target: { x: 0.0, y: 0, z: 0.0 },
  };
  simulator.graphics.lookAt(cameraPosition);

  simulator.run();
});
