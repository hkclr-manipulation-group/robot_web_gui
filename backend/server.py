from flask import Flask, request, jsonify
from ikpy.chain import Chain
import numpy as np

app = Flask(__name__)

chain = Chain.from_urdf_file("urdf/robot.urdf")

@app.route('/api/move_pose', methods=["POST"])
def move_pose():
    pose = request.json
    target_position = [pose['x'], pose['y'], pose['z']]
    angles = chain.inverse_kinematics(target_position)
    return jsonify({'status': 'success', 'angles': angles.tolist()})

@app.route('/api/current_pose', methods=["GET"])
def get_current_pose():
    current_pose = chain.forward_kinematics(np.zeros(chain.n_joints))
    return jsonify({
        'x': current_pose[0, 3],
        'y': current_pose[1, 3],
        'z': current_pose[2, 3]
    })

if __name__ == "__main__":
    app.run(port=8001)