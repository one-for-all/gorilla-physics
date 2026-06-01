import { createParallelBar } from "gorilla-physics";
import { Simulator } from "../Simulator";
import "../hybrid";

createParallelBar().then((state) => {
  let simulator = new Simulator(null);
  simulator.addHybrid(state);
  simulator.updateHybrid();

  let cameraPosition = {
    eye: { x: 0.0, y: -5.0, z: 1 },
    target: { x: 0.0, y: 0, z: 0 },
  };
  simulator.graphics.lookAt(cameraPosition);

  simulator.run(2, 0);
});
