import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

export class RobotViewer {
  constructor(container) {
    this.container = container;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0a0a0a);

    this.camera = new THREE.PerspectiveCamera(
      50,
      container.clientWidth / container.clientHeight,
      0.01,
      100
    );
    this.camera.position.set(1.8, 1.2, 1.8);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.setSize(container.clientWidth, container.clientHeight);
    container.appendChild(this.renderer.domElement);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 0.5, 0);
    this.controls.update();

    this.robot = null;

    this.#addSceneHelpers();
    this.#animate();
    window.addEventListener('resize', () => this.resize());
  }

  #addSceneHelpers() {
    this.scene.add(new THREE.AmbientLight(0xffffff, 0.9));
    const light = new THREE.DirectionalLight(0xffffff, 1.0);
    light.position.set(2, 3, 2);
    this.scene.add(light);
    this.scene.add(new THREE.GridHelper(4, 40, 0x666666, 0x333333));
    this.scene.add(new THREE.AxesHelper(0.5));
  }

  #animate() {
    requestAnimationFrame(() => this.#animate());
    this.renderer.render(this.scene, this.camera);
  }

  resize() {
    const w = this.container.clientWidth;
    const h = this.container.clientHeight;
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h);
  }

  resetView() {
    this.camera.position.set(1.8, 1.2, 1.8);
    this.controls.target.set(0, 0.5, 0);
    this.controls.update();
  }

  setRobot(robot) {
    this.clearRobot();
    this.robot = robot;
    this.scene.add(robot);
    this.fitCameraToObject(robot);
  }

  clearRobot() {
    if (!this.robot) return;
    this.scene.remove(this.robot);
    this.robot.traverse((obj) => {
      if (obj.geometry) obj.geometry.dispose?.();
      if (obj.material) {
        if (Array.isArray(obj.material)) obj.material.forEach((m) => m.dispose?.());
        else obj.material.dispose?.();
      }
    });
    this.robot = null;
  }

  fitCameraToObject(object) {
    const box = new THREE.Box3().setFromObject(object);
    const size = new THREE.Vector3();
    const center = new THREE.Vector3();
    box.getSize(size);
    box.getCenter(center);

    const maxDim = Math.max(size.x, size.y, size.z, 0.5);
    const dist = maxDim * 2.2;

    this.camera.position.set(center.x + dist, center.y + dist * 0.7, center.z + dist);
    this.controls.target.copy(center);
    this.controls.update();
  }
}
