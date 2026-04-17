import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { TransformControls } from "three/addons/controls/TransformControls.js";

export class RobotViewer {

  constructor(container) {

    this.container = container;

    this.robot = null;
    this.callbacks = {};

    this._lock = false;
    this._dragging = false;

    this._initScene();
    this._initRenderer();
    this._initCamera();
    this._initControls();
    this._initLights();
    this._initHelpers();
    this._initTarget();

    this._resize();

    window.addEventListener("resize", () => this._resize());

    this._animate();

  }

  /* ---------------- Scene ---------------- */

  _initScene() {

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0xf6f6f6);
    this.scene.up.set(0, 0, 1);

  }

  /* ---------------- Renderer ---------------- */

  _initRenderer() {

    this.renderer = new THREE.WebGLRenderer({ antialias: true });

    this.renderer.setPixelRatio(window.devicePixelRatio || 1);
    this.renderer.shadowMap.enabled = true;

    this.container.innerHTML = "";
    this.container.appendChild(this.renderer.domElement);

  }

  /* ---------------- Camera ---------------- */

  _initCamera() {

    this.camera = new THREE.PerspectiveCamera(30, 1, 0.01, 100);
    this.camera.position.set(0, -2, 2);

  }

  /* ---------------- Controls ---------------- */

  _initControls() {

    this.orbit = new OrbitControls(
      this.camera,
      this.renderer.domElement
    );

    this.orbit.enableDamping = true;

    this.orbit.target.set(0, 0, 0.35);
    this.orbit.update();

    this.transform = new TransformControls(
      this.camera,
      this.renderer.domElement
    );

    this.scene.add(this.transform);

    /* disable orbit while dragging */

    this.transform.addEventListener("dragging-changed", (e) => {

      this._dragging = e.value;

      this.orbit.enabled = !e.value;

    });

    /* drag update */

    this.transform.addEventListener("objectChange", () => {

      if (this._lock) return;

      if (!this._dragging) return;

      this._emitTargetPose();

    });

    /* drag end */

    this.transform.addEventListener("mouseUp", () => {

      this._emitTargetPose();

    });

  }

  /* ---------------- Lights ---------------- */

  _initLights() {

    const hemi = new THREE.HemisphereLight(
      0xffffff,
      0x1c2c4b,
      1.15
    );

    this.scene.add(hemi);

    const dir = new THREE.DirectionalLight(
      0xffffff,
      1.05
    );

    dir.position.set(3, -3, 4);
    dir.castShadow = true;

    this.scene.add(dir);

  }

  /* ---------------- Helpers ---------------- */

  _initHelpers() {

    const grid = new THREE.GridHelper(
      4,
      20,
      0xff8800,  // 主线颜色 - 亮橙色
      0xcc6600   // 次线颜色 - 深橙色
    );

    grid.rotateX(Math.PI / 2);

    this.scene.add(grid);

    this.scene.add(new THREE.AxesHelper(0.35));

    const ground = new THREE.Mesh(
      new THREE.PlaneGeometry(6, 6),
      new THREE.MeshBasicMaterial({
        color: 0xf6f6f6,
        side: THREE.DoubleSide,
      })
    );

    ground.position.z = -0.001;
    ground.receiveShadow = true;

    this.scene.add(ground);

  }

  /* ---------------- Target ---------------- */

  _initTarget() {

    const geo = new THREE.SphereGeometry(
      0.02,
      32,
      32
    );

    const mat = new THREE.MeshStandardMaterial({
      color: 0xffffff,
    });

    this.target = new THREE.Mesh(geo, mat);

    this.scene.add(this.target);

    this.transform.attach(this.target);

  }

  /* ---------------- Robot ---------------- */

  setRobot(robot) {

    if (this.robot) this.scene.remove(this.robot);

    this.robot = robot;

    if (robot) this.scene.add(robot);

    this.fitToRobot();

  }

  /* ---------------- Fit camera ---------------- */

  fitToRobot() {

    if (!this.robot) return;

    this.robot.updateMatrixWorld(true);

    const box = new THREE.Box3().setFromObject(this.robot);

    if (box.isEmpty()) return;

    const size = box.getSize(new THREE.Vector3());
    const center = box.getCenter(new THREE.Vector3());

    const radius = Math.max(size.x, size.y, size.z, 0.4);

    this.orbit.target.copy(center);

    this.camera.near = Math.max(0.01, radius / 200);
    this.camera.far = Math.max(20, radius * 20);

    this.camera.position.copy(
      center.clone().add(
        new THREE.Vector3(
          radius * 0,
          -radius * 2.5,
          radius * 0.8
        )
      )
    );

    this.camera.updateProjectionMatrix();

    this.orbit.update();

  }

  /* ---------------- Emit target pose ---------------- */

  _emitTargetPose() {

    if (!this.callbacks?.onTaskMove) return;

    const pose = {

      position: this.target.position.clone(),
      quaternion: this.target.quaternion.clone(),

    };

    this.callbacks.onTaskMove(pose);

  }

  /* ---------------- External pose update ---------------- */

updateTargetPose(pose) {

  // if (!pose) return;

  // this._lock = true;

  // // SE3 pose
  // if (pose.position && pose.quaternion) {

  //   this.target.position.copy(pose.position);
  //   this.target.quaternion.copy(pose.quaternion);

  // }
  // // xyz + rpy pose
  // else if (pose.x !== undefined) {

  //   this.target.position.set(
  //     pose.x || 0,
  //     pose.y || 0,
  //     pose.z || 0
  //   );

  //   const euler = new THREE.Euler(
  //     pose.rx || 0,
  //     pose.ry || 0,
  //     pose.rz || 0
  //   );

  //   this.target.quaternion.setFromEuler(euler);

  // }

  // this._lock = false;

}

  /* ---------------- View reset ---------------- */

  resetView() {

    this.camera.position.set(2.2, -2.0, 1.6);

    this.orbit.target.set(0, 0, 0.35);

    this.orbit.update();

  }

  /* ---------------- Resize ---------------- */

  _resize() {

    const rect = this.container.getBoundingClientRect();

    const width = Math.max(200, rect.width);
    const height = Math.max(320, rect.height);

    this.camera.aspect = width / height;

    this.camera.updateProjectionMatrix();

    this.renderer.setSize(width, height, false);

  }

  /* ---------------- Animation ---------------- */

  _animate() {

    requestAnimationFrame(() => this._animate());

    this.orbit.update();

    this.renderer.render(
      this.scene,
      this.camera
    );

  }

}