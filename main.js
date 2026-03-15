const scene = new THREE.Scene();

const camera = new THREE.PerspectiveCamera(
60,
window.innerWidth/window.innerHeight,
0.1,
100
);

camera.position.z = 3;

const renderer = new THREE.WebGLRenderer();

renderer.setSize(window.innerWidth,window.innerHeight);

document.body.appendChild(renderer.domElement);

const cube = new THREE.Mesh(
new THREE.BoxGeometry(),
new THREE.MeshNormalMaterial()
);

scene.add(cube);

function animate(){

requestAnimationFrame(animate);

cube.rotation.x += 0.01;
cube.rotation.y += 0.01;

renderer.render(scene,camera);

}

animate();