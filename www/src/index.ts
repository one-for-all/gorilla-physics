import {
  createFourBarLinkage,
  createFourBarLinkageWithBase,
  createMockNavbot,
  createNavbot,
  createPusher,
  createFluid2D,
  createSphere,
  createTwoSphere,
  createBiped,
  createLeg,
  createLegFromFoot,
} from "gorilla-physics";
import { Simulator } from "./Simulator";
import "./biped";
import { FloatArray } from "./type";
import { RGBELoader } from "three/examples/jsm/loaders/RGBELoader";
import { Color, EquirectangularReflectionMapping } from "three";

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
  let normal = new FloatArray([Math.sin(angle), 0.0, Math.cos(angle)]);
  let h_ground = -0.05 / 2; // 0.0;
  let alpha = 1.0;
  let mu = 1.0;

  // createFEMBox().then((box) => {
  //   simulator.addDeformable(box);
  // });

  let radius = 0.1;
  createBiped().then((state) => {
    state.addHalfSpace(normal as Float64Array, h_ground);

    //  let controller = gorilla.createLegController();
    let controller = gorilla.createBipedController();
    // let controller = gorilla.createNullController();
    // let controller = gorilla.createPusherController();
    // let controller = gorilla.createBalancingBotController();
    // let controller = gorilla.createFourBarLinkageController();
    // let controller = gorilla.createFourBarLinkageWithBaseController();

    // let controller = gorilla.createNavbotController(1.0 / 600.0);
    let interfaceSimulator = new gorilla.InterfaceSimulator(state, controller);
    let simulator = new Simulator(interfaceSimulator);
    // simulator.addLeg();
    simulator.addBiped();

    let poses = simulator.simulator.poses();
    // simulator.updateLeg(poses);
    simulator.updateBiped(poses);

    // simulator.addSphere("sphere1", 0xff0000, radius);
    // simulator.addSphere("sphere2", 0x00ff00, radius);
    // simulator.addNavbot();
    // simulator.updateNavbot();

    let rgbeLoader = new RGBELoader();
    rgbeLoader.load("studio_small_08_1k.hdr", (hdrTexture) => {
      hdrTexture.mapping = EquirectangularReflectionMapping;
      simulator.graphics.scene.environment = hdrTexture;
      // simulator.graphics.scene.background = hdrTexture;
    });

    // let wheel_radius = 0.037 / 2.0;
    // simulator.addSphere("wheel_left", 0x00ff00, wheel_radius);
    // simulator.addSphere("wheel_right", 0x00ff00, wheel_radius);
    simulator.addPlane(normal, h_ground, 10);

    // simulator.addFluid2D(state);

    // simulator.addFourBarLinkage();
    // simulator.addFourBarLinkageWithBase();
    // simulator.addMockNavbot();
    // simulator.addPusher();

    // simulator.addMesh(0, "mesh0");
    // simulator.addMesh(1, "mesh1");
    // simulator.addMesh(2, "mesh2");
    // simulator.addMesh(3, "mesh3");
    // simulator.addMesh(4, "mesh4");
    // simulator.addMesh(5, "gripper");
    // simulator.addMesh(6, "jaw");

    // simulator.updateMesh(0, "mesh0");
    // simulator.updateMesh(1, "mesh1");
    // simulator.updateMesh(2, "mesh2");
    // simulator.updateMesh(3, "mesh3");
    // simulator.updateMesh(4, "mesh4");
    // simulator.updateMesh(5, "gripper");
    // simulator.updateMesh(6, "jaw");

    // let poses = simulator.simulator.poses();
    // simulator.updateFourBarLinkage(poses);

    // Important: Set initial camera position
    let cameraPosition = {
      eye: { x: 0.0, y: -2.0, z: 1.0 },
      target: { x: 0.0, y: 0, z: h_ground },
    };
    simulator.graphics.lookAt(cameraPosition);

    simulator.run(0);
  });
});
