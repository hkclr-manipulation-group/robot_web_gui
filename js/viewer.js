import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';

export class RobotViewer {
  constructor(container) {
    this.container = container;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0b1222);

    // 设置 Z-up
    this.scene.up.set(0, 0, 1);

    this.camera = new THREE.PerspectiveCamera(50, 1, 0.01, 100);
    this.camera.position.set(2.2, -2.0, 1.6);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(window.devicePixelRatio || 1);
    this.renderer.shadowMap.enabled = true;

    container.innerHTML = '';
    container.appendChild(this.renderer.domElement);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;

    // 目标控制
    this.transformControls = new TransformControls(this.camera, this.renderer.domElement);
    this.scene.add(this.transformControls);

    // 监听 TransformControls 事件
    this.transformControls.addEventListener('change', this.onTransformChange.bind(this));

    this.controls.target.set(0, 0, 0.35);
    this.controls.update();

    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x1c2c4b, 1.15));
    const dir = new THREE.DirectionalLight(0xffffff, 1.05);
    dir.position.set(3, -3, 4);
    dir.castShadow = true;
    this.scene.add(dir);

    const grid = new THREE.GridHelper(4, 20, 0x3f6eb8, 0x263b64);
    grid.rotateX(Math.PI / 2);
    this.scene.add(grid);
    this.scene.add(new THREE.AxesHelper(0.35));

    const ground = new THREE.Mesh(
      new THREE.PlaneGeometry(6, 6),
      new THREE.MeshPhongMaterial({ color: 0x101a30, side: THREE.DoubleSide })
    );
    ground.position.z = -0.001;
    ground.receiveShadow = true;
    this.scene.add(ground);

    this.robot = null;

    this._animate = this._animate.bind(this);
    this._resize = this._resize.bind(this);

    this._resize();
    window.addEventListener('resize', this._resize);

    requestAnimationFrame(this._animate);
  }

  // 末端执行器拖拽时的变化回调
  onTransformChange() {
    if (this.callbacks?.onMove) {
      const targetPose = this.transformControls.object.position;
      this.callbacks.onMove(targetPose);
    }
  }

  setRobot(robot) {
    if (this.robot) this.scene.remove(this.robot);
    this.robot = robot;
    if (robot) this.scene.add(robot);

    // 假设机器人末端执行器是 robot.tipLink
    this.transformControls.attach(robot.tipLink);  // 确保绑定的是末端执行器
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
    this.camera.position.copy(center.clone().add(new THREE.Vector3(radius * 2.0, -radius * 1.8, radius * 1.2)));
    this.camera.updateProjectionMatrix();
    this.controls.update();
  }

  resetView() {
    this.camera.position.set(2.2, -2.0, 1.6);
    this.controls.target.set(0, 0, 0.35);
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
}