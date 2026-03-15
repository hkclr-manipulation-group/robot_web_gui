import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

export class RobotViewer {
  constructor(container) {
    this.container = container;

    // 创建场景，设置背景色
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0b1222);

    // 关键：设置场景使用 Z-up 坐标系
    this.scene.up.set(0, 0, 1);  // 设置世界坐标系的上方向为 Z 轴

    // 设置相机
    this.camera = new THREE.PerspectiveCamera(50, 1, 0.01, 100);
    this.camera.position.set(2.2, -2.0, 1.6);  // 适应 Z-up 的相机位置

    // 设置渲染器
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(window.devicePixelRatio || 1);
    this.renderer.shadowMap.enabled = true;

    container.innerHTML = '';
    container.appendChild(this.renderer.domElement);

    // 创建 OrbitControls 控制器
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;

    // 设置 OrbitControls 目标
    this.controls.target.set(0, 0, 0.35);
    this.controls.update();

    // 添加环境光源
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x1c2c4b, 1.15));

    // 添加方向光源
    const dir = new THREE.DirectionalLight(0xffffff, 1.05);
    dir.position.set(3, -3, 4);  // 适应 Z-up
    dir.castShadow = true;
    this.scene.add(dir);

    // 添加 GridHelper (旋转到合适位置)
    const grid = new THREE.GridHelper(4, 20, 0x3f6eb8, 0x263b64);
    // grid.rotateX(Math.PI / 2);  // 旋转到 XY 面
    this.scene.add(grid);

    // 添加坐标轴
    this.scene.add(new THREE.AxesHelper(0.35));

    // 添加地面 (Z-up)
    const ground = new THREE.Mesh(
      new THREE.PlaneGeometry(6, 6),
      new THREE.MeshPhongMaterial({
        color: 0x101a30,
        side: THREE.DoubleSide
      })
    );
    ground.position.z = -0.001;  // 轻微偏移，让它接触地面
    ground.receiveShadow = true;
    this.scene.add(ground);

    this.robot = null;

    // 绑定动画和窗口重尺寸
    this._animate = this._animate.bind(this);
    this._resize = this._resize.bind(this);

    this._resize();
    window.addEventListener('resize', this._resize);

    // 启动动画循环
    requestAnimationFrame(this._animate);
  }

  // 设置机器人模型
  setRobot(robot) {
    if (this.robot) this.scene.remove(this.robot);
    this.robot = robot;
    if (robot) this.scene.add(robot);
    this.fitToRobot();
  }

  // 自动适配机器人视角
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

    // 设置相机位置，适配 Z-up
    this.camera.position.copy(center.clone().add(new THREE.Vector3(radius * 2.0, -radius * 1.8, radius * 1.2)));
    this.camera.updateProjectionMatrix();
    this.controls.update();
  }

  // 重置视角
  resetView() {
    this.camera.position.set(2.2, -2.0, 1.6);
    this.controls.target.set(0, 0, 0.35);
    this.controls.update();
  }

  // 窗口调整时重新设置尺寸
  _resize() {
    const rect = this.container.getBoundingClientRect();
    const width = Math.max(200, rect.width);
    const height = Math.max(320, rect.height);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height, false);
  }

  // 动画循环
  _animate() {
    requestAnimationFrame(this._animate);
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }
}