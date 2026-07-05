import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls";
import { RoomEnvironment } from "three/examples/jsm/environments/RoomEnvironment";

export class Graphics {
  scene: THREE.Scene;
  camera: THREE.PerspectiveCamera;
  renderer: THREE.WebGLRenderer;
  light: THREE.PointLight;
  controls: OrbitControls;
  onResize?: () => void;

  constructor(showGrid: boolean = true) {
    this.scene = new THREE.Scene();

    let view_div = document.getElementById("threejs").getBoundingClientRect();
    this.camera = new THREE.PerspectiveCamera(
      45,
      view_div.width / view_div.height,
      0.1,
      10000,
    );
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setSize(view_div.width, view_div.height);
    this.renderer.setClearColor(0x292929, 1);
    // this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    // High pixel Ratio make the rendering extremely slow, so we cap it.
    const pixelRatio = window.devicePixelRatio
      ? Math.min(window.devicePixelRatio, 1.5)
      : 1;
    this.renderer.setPixelRatio(pixelRatio);
    const container = document.getElementById("threejs");
    container.appendChild(this.renderer.domElement);

    let ambientLight = new THREE.AmbientLight(0x606060);
    // let ambientLight = new THREE.AmbientLight(0xffffff);
    this.scene.add(ambientLight);
    this.light = new THREE.PointLight(0xffffff, 1, 1000);
    this.scene.add(this.light);

    // Room Environment
    // const pmremGenerator = new THREE.PMREMGenerator(this.renderer);
    // const envMap = pmremGenerator.fromScene(new RoomEnvironment()).texture;
    // this.scene.environment = envMap;

    // TODO: optionally show grid
    if (showGrid) {
      const gridHelper = new THREE.GridHelper(10, 100, 0xff0000, 0x0000ff); // Red center line, blue grid lines
      gridHelper.rotateOnAxis(new THREE.Vector3(1, 0, 0), -Math.PI / 2);
      // gridHelper.translateY(0.05 / 2);
      this.scene.add(gridHelper);
    }

    // Keep the canvas filling the #threejs container. On desktop a plain
    // window 'resize' is enough, but on iOS Safari the container's height
    // (100dvh) grows/shrinks as the browser toolbar hides/shows, and that
    // does NOT reliably fire window 'resize' — it fires visualViewport
    // 'resize'/'scroll' instead. If we miss those, the canvas stays shorter
    // than the container and the page background shows through beneath it.
    // So re-measure the container on all of these signals.
    let me = this;
    let lastWidth = Math.round(view_div.width);
    let lastHeight = Math.round(view_div.height);
    let rafId = 0;

    function applyResize() {
      rafId = 0;
      if (!me.camera) return;
      const rect = document.getElementById("threejs").getBoundingClientRect();
      const width = Math.round(rect.width);
      const height = Math.round(rect.height);
      // Skip redundant work: reallocating the drawing buffer on every event
      // during the toolbar animation is expensive, so only resize when the
      // measured size actually changes.
      if (width === lastWidth && height === lastHeight) return;
      lastWidth = width;
      lastHeight = height;
      me.camera.aspect = width / height;
      me.camera.updateProjectionMatrix();
      me.renderer.setSize(width, height);
      // Repaint immediately so the freshly-sized (and just-cleared) buffer
      // isn't left blank until the next sim-driven render frame.
      me.renderer.render(me.scene, me.camera);
    }

    // Collapse bursts of viewport events into a single resize per frame.
    function scheduleResize() {
      if (rafId) return;
      rafId = requestAnimationFrame(applyResize);
    }

    this.onResize = scheduleResize;
    window.addEventListener("resize", scheduleResize, false);
    window.addEventListener("orientationchange", scheduleResize, false);
    if (window.visualViewport) {
      window.visualViewport.addEventListener("resize", scheduleResize);
      window.visualViewport.addEventListener("scroll", scheduleResize);
    }

    // Customize control's rotation axis
    this.camera.up.set(0, 0, 1);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = false;
  }

  dispose() {
    if (this.onResize) {
      window.removeEventListener("resize", this.onResize, false);
      window.removeEventListener("orientationchange", this.onResize, false);
      if (window.visualViewport) {
        window.visualViewport.removeEventListener("resize", this.onResize);
        window.visualViewport.removeEventListener("scroll", this.onResize);
      }
      this.onResize = undefined;
    }
  }

  render() {
    this.controls.update();

    // Note: setting light position every time camera position changes is
    // rather expensive as renderer needs to re-render the scene.
    // Consider putting 4 fixed light around the model.

    this.light.position.set(
      this.camera.position.x,
      this.camera.position.y,
      this.camera.position.z,
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
