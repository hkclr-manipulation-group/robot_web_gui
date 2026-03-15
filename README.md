# Robot Web GUI

Static browser-based robot GUI for GitHub Pages.

## Included features

- URDF loading from repository path or local file
- Automatic joint slider generation from URDF joints
- End-effector 6D pose display
- Task-space control with browser-side numerical IK
- Joint trajectory planning
- Cartesian trajectory planning via IK
- Teach / record current pose
- Save and load trajectory JSON
- Path playback in browser preview mode

## Important limitation

This repo runs fully on the client side, so on GitHub Pages it can preview and simulate robot motion in the browser, but it **cannot directly control a local robot arm** unless you connect it to a separate backend service and replace `js/api.js`.

## Run locally

Because ES modules and URDF fetching need HTTP, use a local server.

```bash
python -m http.server 8000
```

Then open:

```text
http://127.0.0.1:8000/
```

## Deploy to GitHub Pages

1. Create a repo.
2. Upload all files in this folder.
3. Keep `.nojekyll` in the repository root.
4. Enable GitHub Pages from the main branch / root.

## Files to customize

- `assets/urdf/simple6dof/simple6dof.urdf`: sample robot
- `js/api.js`: replace preview API with your backend
- `js/kinematics.js`: browser IK and kinematics logic

## Notes on IK

The IK solver is numerical and generic enough for simple serial-chain URDFs, but it is not a full MoveIt replacement. For production hardware control, add:

- collision checking
- joint limit handling inside IK iteration
- singularity handling
- velocity / acceleration constraints
- backend safety interlocks
