const scene = new THREE.Scene();

const camera = new THREE.PerspectiveCamera(
    60,
    window.innerWidth/window.innerHeight,
    0.01,
    100
);

camera.position.set(1,1,1);

const renderer = new THREE.WebGLRenderer({antialias:true});
renderer.setSize(window.innerWidth, window.innerHeight);

document.getElementById("viewer").appendChild(renderer.domElement);

const controls = new THREE.OrbitControls(camera, renderer.domElement);

const light = new THREE.DirectionalLight(0xffffff,1);
light.position.set(5,5,5);
scene.add(light);

scene.add(new THREE.AmbientLight(0x666666));

const grid = new THREE.GridHelper(2,20);
scene.add(grid);

let robot;

const loader = new URDFLoader();

loader.load(
    "./urdf/robot.urdf",
    function(result){

        robot = result;
        scene.add(robot);

        initGUI();
    }
);

function initGUI(){

    const gui = new dat.GUI();

    const joints = {};

    robot.traverse(obj=>{
        if(obj.isURDFJoint){

            joints[obj.name] = 0;

            gui.add(joints,obj.name,-3.14,3.14)
            .onChange(v=>{
                obj.setJointValue(v);
            });

        }
    });

    const task = {

        x:0,
        y:0,
        z:0.3,

        move:()=>{

            if(!robot) return;

            const ee = robot.getObjectByName("ee_link");

            ee.position.set(
                task.x,
                task.y,
                task.z
            );

        }

    };

    const taskFolder = gui.addFolder("Task Space");

    taskFolder.add(task,"x",-0.5,0.5);
    taskFolder.add(task,"y",-0.5,0.5);
    taskFolder.add(task,"z",0,0.8);
    taskFolder.add(task,"move");

}

function animate(){

    requestAnimationFrame(animate);

    renderer.render(scene,camera);

}

animate();