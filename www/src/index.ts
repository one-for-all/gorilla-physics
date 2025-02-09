import { Simulator } from "./Simulator";

import("gorilla-physics").then((gorilla) => {
  const mass = 1.0;
  const radius = 2.0;
  const length = 10.0;
  let w_body = 5.0;
  let h_body = 0.1;
  let r_leg = 0.5;
  let r_foot = 0.5;
  let body_leg_length = 2.0;
  let leg_foot_length = 10.0;
  let mechanismState = gorilla.createHopper(
    w_body,
    h_body,
    r_leg,
    r_foot,
    body_leg_length,
    leg_foot_length
  );

  let normal = new Float32Array([0.0, 0.0, 1.0]);
  let distance = -20.0;
  mechanismState.addHalfSpace(normal, distance);

  let h_setpoint = 0.0;
  let controller = gorilla.createHopperController(
    h_setpoint,
    body_leg_length,
    leg_foot_length
  );

  let interfaceSimulator = new gorilla.InterfaceSimulator(
    mechanismState,
    controller
  );

  let simulator = new Simulator(interfaceSimulator);
  // simulator.addCart(length);
  // simulator.addCartPole(length);
  // simulator.addDoublemPendulum(length);
  // simulator.addPendulum(length);
  // simulator.addSphere(radius);
  simulator.addHopper(w_body, h_body, r_leg, r_foot);

  simulator.addPlane(normal, distance, 100);

  // Important: Set initial camera position
  let cameraPosition = {
    eye: { x: 0.0, y: -50.0, z: 0.0 },
    target: { x: 0.0, y: 0, z: 0.0 },
  };
  simulator.graphics.lookAt(cameraPosition);

  simulator.run();
});
