import { Simulator } from "./Simulator";

import("gorilla-physics").then((gorilla) => {
  const mass = 1.0;
  const radius = 2.0;
  const length = 10.0;
  // let mechanismState = gorilla.createSimplePendulum(mass, length);
  let mechanismState = gorilla.createSphere(mass, radius);

  // let point = new Float32Array([0.0, 0.0, -5.0]);
  let normal = new Float32Array([0.0, 0.0, 1.0]);
  let distance = -10.0;
  mechanismState.addHalfSpace(normal, distance);

  // let mechanismState = gorilla.createDoublePendulum(length);
  // let mechanismState = gorilla.createCart(length);
  // let mechanismState = gorilla.createCartPole(length);
  let simulator = new Simulator(mechanismState);
  // simulator.addCart(length);
  // simulator.addCartPole(length);
  // simulator.addDoublemPendulum(length);
  // simulator.addPendulum(length);
  simulator.addPlane(normal, distance, 100);
  simulator.addSphere(radius);

  // Important: Set initial camera position
  let cameraPosition = {
    eye: { x: 0.0, y: 10.0, z: 20.0 },
    target: { x: 0.0, y: 0, z: 0.0 },
  };
  simulator.graphics.lookAt(cameraPosition);

  simulator.run();
});
