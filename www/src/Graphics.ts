import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls";

export class Graphics {
  scene: THREE.Scene;
  camera: THREE.PerspectiveCamera;
  renderer: THREE.WebGLRenderer;
  light: THREE.PointLight;
  controls: OrbitControls;

  constructor() {
    this.scene = new THREE.Scene();

    let view_div = document.getElementById("threejs").getBoundingClientRect();
    this.camera = new THREE.PerspectiveCamera(
      45,
      view_div.width / view_div.height,
      0.1,
      10000
    );
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setSize(view_div.width, view_div.height);
    this.renderer.setClearColor(0x292929, 1);
    // High pixel Ratio make the rendering extremely slow, so we cap it.
    const pixelRatio = window.devicePixelRatio
      ? Math.min(window.devicePixelRatio, 1.5)
      : 1;
    this.renderer.setPixelRatio(pixelRatio);
    document.getElementById("threejs").appendChild(this.renderer.domElement);

    let ambientLight = new THREE.AmbientLight(0x606060);
    this.scene.add(ambientLight);
    this.light = new THREE.PointLight(0xffffff, 1, 1000);
    this.scene.add(this.light);

    const gridHelper = new THREE.GridHelper(50, 500, 0xff0000, 0x0000ff); // Red center line, blue grid lines
    gridHelper.rotateOnAxis(new THREE.Vector3(1, 0, 0), -Math.PI / 2);
    this.scene.add(gridHelper);

    let me = this;
    function onWindowResize() {
      let _view_div = document
        .getElementById("threejs")
        .getBoundingClientRect();
      if (!!me.camera) {
        me.camera.aspect = _view_div.width / _view_div.height;
        me.camera.updateProjectionMatrix();
        me.renderer.setSize(_view_div.width, _view_div.height);
      }
    }
    window.addEventListener("resize", onWindowResize, false);

    // Customize control's rotation axis
    this.camera.up.set(0, 0, 1);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = false;
  }

  render() {
    this.controls.update();

    // Note: setting light position every time camera position changes is
    // rather expensive as renderer needs to re-render the scene.
    // Consider putting 4 fixed light around the model.
    this.light.position.set(
      this.camera.position.x,
      this.camera.position.y,
      this.camera.position.z
    );

    this.renderer.render(this.scene, this.camera);
  }

  lookAt(pos: {
    target: { x: number; y: number; z: number };
    eye: { x: number; y: number; z: number };
  }) {
    this.camera.position.set(pos.eye.x, pos.eye.y, pos.eye.z);
    this.controls.target.set(pos.target.x, pos.target.y, pos.target.z);
    this.controls.update();
  }
}
