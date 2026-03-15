# URDF Robot Web GUI

A minimal modular web GUI for loading a URDF, visualizing the robot in Three.js, and controlling joints from the browser.

## Project structure

```text
urdf_web_gui/
├── index.html
├── styles.css
├── server.py
├── js/
│   ├── api.js
│   ├── config.js
│   ├── joints-ui.js
│   ├── main.js
│   ├── urdf-loader-wrapper.js
│   ├── utils.js
│   └── viewer.js
└── urdf/
    └── robot.urdf
```

## Run

Use Python:

```bash
python server.py
```

or:

```bash
python -m http.server 8000
```

Then open:

```text
http://127.0.0.1:8000
```

## Important notes

- Replace `./urdf/robot.urdf` with your real URDF.
- Prefer relative mesh paths inside the URDF.
- `server.py` includes dummy `/api/*` endpoints for quick testing.
- To connect a real robot, edit `js/api.js`.

## Common URDF mesh paths

Recommended:

```xml
<mesh filename="meshes/link1.stl"/>
```

Use caution with package paths:

```xml
<mesh filename="package://my_robot_description/meshes/link1.stl"/>
```

If you use package paths, you may need custom path rewriting in `js/urdf-loader-wrapper.js`.
