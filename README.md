# Robot Web GUI

A GitHub Pages friendly robot GUI with these pieces already wired together:

- sticky 3D robot viewer
- scrollable right-side control panel
- URDF loading
- joint-space control with sliders
- 6-DoF task-space control with sliders
- browser-side numerical IK
- teach / playback via Spark2 SDK (`start_teach` / `reset_playback` / `start_playback`)
- multi-robot selector for WiFi robot setups
- local gateway example for forwarding commands to real robots

## Folder structure

- `index.html`, `styles.css`: UI layout and styling
- `js/`: viewer, UI, IK, planner, teach, storage, gateway API adapter
- `urdf/robot.urdf`: default Spark2 URDF for preview and control
- `server_examples/robot_gateway_server.py`: tiny local HTTP gateway example
- `.nojekyll`: allows GitHub Pages to serve files directly

## GitHub Pages deployment

1. Upload the whole project to a GitHub repository.
2. Enable GitHub Pages for the repository root or `/docs` equivalent.
3. Open the generated site URL.
4. The default mode is preview-only. The GUI will still animate the robot locally.

## Real robot over WiFi

GitHub Pages cannot directly host your robot control backend. Use a local gateway service on the same LAN as the robots.

- frontend: `https://<your-user>.github.io/<repo>`
- local gateway: `http://<your-pc-ip>:9000`
  
Example:
- frontend:https://hkclr-manipulation-group.github.io/robot_web_gui/
- local gateway: http://192.168.3.11:9000

Local host:

1. Open a terminal in this project directory (`robot_web_gui`).
2. Start a simple static file server:
```bash
python3 -m http.server 8000
```
3. Open `http://localhost:8000` in your browser.


In the GUI:

1. set `Gateway URL`
2. choose the active robot from the robot selector
3. click `Connect`
4. move joints or send task-space commands

## Notes

- The included IK is a generic damped least-squares numerical solver, meant for lightweight web control and preview.
- Collision checking is not included.
- For a production system, replace the demo gateway logic with your real robot protocol, safety checks, and watchdogs.
  Example: python3 server_examples/rt_control_server.py
