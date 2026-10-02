# Dev container and SIL viewers

The same `.devcontainer/devcontainer.json` is used on WSL and on the Jetson (Remote-SSH, then
"Reopen in Container"). Everything that differs between hosts is discovered by
`.devcontainer/initialize.sh`, which VS Code runs on the host before each container start and which
writes `.devcontainer/.generated/` (git-ignored):

- `host.env`: `DISPLAY` of the host, or nothing if the host has no X server.
- `Xauthority`: the host's X11 cookies rewritten to match any hostname (the container's hostname
  differs from the host's), or an empty file for WSLg, which does not authenticate.

The script's one-line summary appears in the "Dev Containers" output. The container reads these when
it is **created**: after the host's display changes, run "Dev Containers: Rebuild Container".

## Viewing the SIL

`sil_stand.launch.py` takes two switches:

| Argument | Default | Effect |
|---|---|---|
| `rviz` | `true` | Start RViz2. Needs an X display; on a remote host that means X forwarding, which is slow. |
| `foxglove` | `false` | Start `foxglove_bridge` on port 8765. Rendering happens in the viewer, so the robot needs no display or GPU. |

```bash
ros2 launch humanoid_bringup sil_stand.launch.py                           # RViz2 (WSL)
ros2 launch humanoid_bringup sil_stand.launch.py rviz:=false foxglove:=true  # Foxglove (Jetson)
```

Both can be enabled together. The bridge is read-only (`connectionGraph` and `assets` only: no
client publishing, services or parameters).

### Foxglove setup (once)

1. Open Foxglove (desktop or app.foxglove.dev) and "Open connection" → "Foxglove WebSocket" →
   `ws://localhost:8765`. VS Code forwards port 8765 from the Jetson; `ws://10.88.111.29:8765` also
   works from the lab network.
2. Add a 3D panel. Under Custom layers, add a URDF layer: source "Topic",
   `/ground_truth/robot_description`. Set the display frame to `sim_world`.
3. Save the layout.

The bridge side is tested (topics advertised, mesh served). The Foxglove UI steps above were written
from the documented UI and not exercised against a running viewer; adjust them if the menus differ.

`robot.urdf` refers to its meshes as `package://humanoid_bringup/model/source/...`. Foxglove fetches
only `package://` meshes over the bridge (`file://` is read from the viewer's own disk), and the
bridge's default allow-list serves them from the installed `humanoid_bringup` share directory.
