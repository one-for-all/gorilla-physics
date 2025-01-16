import { Simulator } from "./Simulator";

import("gorilla-physics").then((gorilla) => {
  const length = 10.0;
  let mechanismState = gorilla.createDoublePendulum(length);
  // let mechanismState = gorilla.createCart(length);
  // let mechanismState = gorilla.createCartPole(length);
  let simulator = new Simulator(mechanismState);
  // simulator.addCart(length);
  // simulator.addCartPole(length);
  simulator.addDoublemPendulum(length);

  // Important: Set initial camera position
  let cameraPosition = {
    eye: { x: 0.0, y: 10.0, z: 20.0 },
    target: { x: 0.0, y: 0, z: 0.0 },
  };
  simulator.graphics.lookAt(cameraPosition);

  simulator.run();
});
