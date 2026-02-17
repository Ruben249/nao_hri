nao_bt_controller — README (Markdown)
Generated on 2026-02-17 — copy/paste the Markdown block below.
# nao_bt_controller
Behaviour-Tree (BT) runner and orchestration utilities for NAO HRI demos.
This package provides the **BT runner script** used to drive a multi-node HRI pipeline by publishing
---
## Contents
- [What this package does](#what-this-package-does)
- [Repository layout](#repository-layout)
- [ROS interfaces](#ros-interfaces)
- [Dependencies](#dependencies)
- [Build](#build)
- [Run](#run)
- [Parameters](#parameters)
- [Troubleshooting](#troubleshooting)
- [License](#license)
---
## What this package does
`nao_bt_controller` contains a Python-based **BT runner** that:
- executes a predefined behaviour tree to coordinate an HRI interaction loop
- publishes commands to HRI “bridges/stubs” through a topic-based contract under
- blocks/advances the tree based on the result topics (success/failure/running),
- centralizes timing (timeouts / attempt windows) and sequencing at the BT level.
Entry point:
- [`nao_bt_controller/bt_runner.py`](nao_bt_controller/bt_runner.py)
---
## Repository layout
```
nao_bt_controller/
├── nao_bt_controller/
│
├── __init__.py
│
└── bt_runner.py
├── resource/
│
└── nao_bt_controller
├── package.xml
├── setup.cfg
└── setup.py
```

---
## ROS interfaces
The BT runner acts as a **coordinator**, typically using the following topic contract
### Command topics (published by BT runner)
- `/hri/ui_cmd` – UI state changes (mode, text, video, duration)- `/hri/speak_cmd` – request TTS / speech playback
- `/hri/nao_pos_cmd` – request a NAO posture/pose execution
- `/hri/eval_cmd` – request an evaluation step (e.g., pose validation)
### Result topics (subscribed by BT runner)
- `/hri/ui_result`
- `/hri/speak_result`
- `/hri/nao_pos_result`
- `/hri/eval_result`
Result payload convention:
- `"running"` while executing
- `"success"` or `"failure"` when the step completes
> The BT runner itself does not implement the UI, speech, pose, or evaluation logic.
---
## Dependencies
### Runtime
- ROS 2 (tested with **Rolling**)
- `rclpy`
- Standard Python 3 runtime used by your ROS 2 environment
### Expected external nodes
For a complete demo run, the BT runner expects other processes to provide the `/hri/*` endpoints
---
## Build
From your ROS 2 workspace root (e.g., `nao_ws`):
```bash
source /opt/ros/rolling/setup.bash
colcon build --packages-select nao_bt_controller
source install/setup.bash
```
---
## Run
From the workspace root:
```bash
source /opt/ros/rolling/setup.bash
source install/setup.bash
python3 src/thirdparty/nao_hri/nao_bt_controller/nao_bt_controller/bt_runner.py
```
If you installed the package as an entry point, you can also run it with:
```bash
ros2 run nao_bt_controller bt_runner
```
(Only works if an entry point named `bt_runner` is defined in `setup.py`.)
---
## Parameters
The BT runner is typically driven by ROS parameters such as:
- `catalog_path` – path to the exercises catalog YAML used by the BT logic
- `pose_id` – selected exercise/pose identifier
- `attempt_window_s` – time window for user attempt
- `eval_timeout_s` – maximum evaluation time before failingExample:
```bash
python3 src/thirdparty/nao_hri/nao_bt_controller/nao_bt_controller/bt_runner.py \
--ros-args -p pose_id:=rightarmUp -p attempt_window_s:=6.0
```
> Note: keep `catalog_path` consistent with your workspace layout (it can be a workspace-relative path)
---

The other panes (UI stub, speak bridge, evaluators, etc.) are **external dependencies** and belong to
---
## Troubleshooting
### BT runner never progresses (stuck in "running")
- Verify the expected `/hri/*_result` topics are being published by the corresponding nodes.
- Check that topic names match exactly (namespace and remaps).
### Immediate failures
- Confirm that the downstream nodes interpret the command payload format your BT is publishing (e.g.,
- Validate parameter values (timeouts too small are a common cause).
### Quick introspection commands
```bash
ros2 topic list | grep "^/hri/"
ros2 topic echo /hri/ui_cmd
ros2 topic echo /hri/ui_result
```
---
## License
See the repository-level `LICENSE` file (if present) or the license declaration in [`package.xml`](pa