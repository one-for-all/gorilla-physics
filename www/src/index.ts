import { Simulator } from "./Simulator";

import("gorilla-physics").then((gorilla) => {
  const mass = 1.0;
  const radius = 5.0;
  const length = 5.0;
  let w_body = 3.5;
  let h_body = 0.05;
  let r_hip = 0.1;
  let r_piston = 0.1;
  let l_leg = 1.0;
  let body_hip_length = 0.0;
  let body_leg_length = 2.0;
  let hip_piston_length = 0.0;
  let piston_leg_length = 1.0;

  let k_spring = 1e3;
  // let state = gorilla.create1DHopper(
  //   w_body,
  //   h_body,
  //   r_leg,
  //   r_foot,
  //   body_leg_length,
  //   leg_foot_length
  // );
  // let state = gorilla.createCube(length);
  // let state = gorilla.create2DHopper(
  //   w_body,
  //   h_body,
  //   r_hip,
  //   r_leg,
  //   r_foot,
  //   body_hip_length,
  //   hip_leg_length,
  //   leg_foot_length
  // );
  // let state = gorilla.create2DHopper2(
  //   w_body,
  //   h_body,
  //   r_hip,
  //   body_hip_length,
  //   r_piston,
  //   hip_piston_length,
  //   l_leg,
  //   piston_leg_length
  // );

  let state = gorilla.createSLIP();

  let n_foot = 8;
  // let state = gorilla.createRimlessWheel(radius, n_foot);

  let angle: number = (0.0 * Math.PI) / 180.0;
  let normal = new Float32Array([Math.sin(angle), 0.0, Math.cos(angle)]);
  let distance = -0.5;
  state.addHalfSpace(normal, distance);

  let h_setpoint = 0.0;
  // let controller = gorilla.createHopper2DController(
  //   h_setpoint,
  //   body_hip_length,
  //   hip_piston_length,
  //   piston_leg_length,
  //   l_leg,
  //   k_spring
  // );
  // let controller = gorilla.createHopper1DController(
  //   h_setpoint,
  //   body_hip_length,
  //   leg_foot_length
  // );
  let controller = gorilla.createNullController();

  let interfaceSimulator = new gorilla.InterfaceSimulator(state, controller);

  let simulator = new Simulator(interfaceSimulator);
  // simulator.addCart(length);
  // simulator.addCartPole(length);
  // simulator.addDoublemPendulum(length);
  // simulator.addPendulum(length);
  // simulator.addSphere(radius);
  // simulator.add2DHopper(w_body, h_body, r_hip, r_piston, r_foot);
  // simulator.add2DHopper2(w_body, h_body, r_hip, r_piston, l_leg);
  // simulator.add1DHopper(w_body, h_body, r_leg, r_foot);
  // simulator.addCube(length);
  simulator.addSphere("sphere", 0xffffff, 0.1);
  // simulator.addRimlessWheel(radius, 10.0, n_foot);

  simulator.addPlane(normal, distance, 100);

  // Important: Set initial camera position
  let cameraPosition = {
    eye: { x: 0.0, y: -10.0, z: -0.0 },
    target: { x: 0.0, y: 0, z: 0.0 },
  };
  simulator.graphics.lookAt(cameraPosition);

  simulator.run();
});
