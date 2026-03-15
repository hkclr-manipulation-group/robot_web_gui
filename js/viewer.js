import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

export class RobotViewer {
  constructor(container) {
    this.container = container;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0b1222);

    this.camera = new THREE.PerspectiveCamera(50, 1, 0.01, 100);
    this.camera.position.set(1.8, 1.3, 1.8);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(window.devicePixelRatio || 1);
    this.renderer.shadowMap.enabled = true;
    container.innerHTML = '';
    container.appendChild(this.renderer.domElement);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.target.set(0, 0.35, 0);
    this.controls.update();

    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x1c2c4b, 1.15));

    const dir = new THREE.DirectionalLight(0xffffff, 1.05);
    dir.position.set(2, 3, 2);
    dir.castShadow = true;
    this.scene.add(dir);

    this.scene.add(new THREE.GridHelper(4, 20, 0x3f6eb8, 0x263b64));
    this.scene.add(new THREE.AxesHelper(0.35));

    const ground = new THREE.Mesh(
      new THREE.PlaneGeometry(6, 6),
      new THREE.MeshPhongMaterial({ color: 0x101a30, side: THREE.DoubleSide })
    );
    ground.rotation.x = -Math.PI / 2;
    ground.position.y = -0.001;
    ground.receiveShadow = true;
    this.scene.add(ground);

    this.robot = null;
    this._animate = this._animate.bind(this);
    this._resize = this._resize.bind(this);
    this._resize();
    window.addEventListener('resize', this._resize);
    requestAnimationFrame(this._animate);
  }

  setRobot(robot) {
    if (this.robot) this.scene.remove(this.robot);
    this.robot = robot;
    if (robot) this.scene.add(robot);
    this.fitToRobot();
  }

  fitToRobot() {
    if (!this.robot) return;
    this.robot.updateMatrixWorld(true);
    const box = new THREE.Box3().setFromObject(this.robot);
    if (box.isEmpty()) return;
    const size = box.getSize(new THREE.Vector3());
    const center = box.getCenter(new THREE.Vector3());
    const radius = Math.max(size.x, size.y, size.z, 0.4);
    this.controls.target.copy(center);
    this.camera.near = Math.max(0.01, radius / 200);
    this.camera.far = Math.max(20, radius * 20);
    this.camera.position.copy(center.clone().add(new THREE.Vector3(radius * 1.8, radius * 1.25, radius * 1.8)));
    this.camera.updateProjectionMatrix();
    this.controls.update();
  }

  resetView() {
    this.camera.position.set(1.8, 1.3, 1.8);
    this.controls.target.set(0, 0.35, 0);
    this.controls.update();
  }

  _resize() {
    const rect = this.container.getBoundingClientRect();
    const width = Math.max(200, rect.width);
    const height = Math.max(320, rect.height);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height, false);
  }

  _animate() {
    requestAnimationFrame(this._animate);
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }
}
