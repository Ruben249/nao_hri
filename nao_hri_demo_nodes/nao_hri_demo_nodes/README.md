# nao_hri_demo_nodes
Helper/demo ROS 2 nodes that connect the Behaviour Tree controller with external subsystems
(Web UI, LEDs, speech, and evaluation) using a simple topic-based HRI interface under
`/hri/*`.
This package is designed to be used inside the **NAO_HRI** workspace.
---
## Contents
- [What is in this package](#what-is-in-this-package)
- [Companion processes (required to run)](#companion-processes-required-to-run)
- [Nodes and scripts](#nodes-and-scripts)
- [`ui_stub`](#ui_stub)
- [`nao_pos_bridge.py`](#nao_pos_bridgepy)
- [`opencv_eval_node.py`](#opencv_eval_nodepy)
- [`speak_bridge.py`](#speak_bridgepy)
- [Interfaces](#interfaces)
- [Dependencies](#dependencies)
- [Build](#build)
- [Run](#run)
- [Configuration](#configuration)
- [Troubleshooting](#troubleshooting)
- [License](#license)
---
## What is in this package
`nao_hri_demo_nodes` provides small **adapter nodes** ("stubs/bridges") so the BT runner can
remain decoupled from implementation details.
In practice you:
1. publish a JSON command on a `/hri/*_cmd` topic (from the BT),
2. the bridge talks to a service/action server,
3. the bridge publishes `/hri/*_result` so the BT can progress.
Repository layout (as in the workspace tree):
- `nao_hri_demo_nodes/` (package root)
- `nao_hri_demo_nodes/scripts/` (Python nodes)
- `nao_hri_demo_nodes/config/` (runtime configs, templates, etc.)
> GitHub package path (relative): `nao_hri/nao_hri_demo_nodes/nao_hri_demo_nodes`
---
## Companion processes (required to run)
This package is not a full "bringup" by itself: most nodes here **bridge** the `/hri/*`
topic interface to other services/actions. To run the full demo stack you typically need:
### Web UI + ROS bridge (required for `ui_stub`)
`ui_stub` calls the service:
- `/robot/ui/command` (`server_web_interfaces/srv/UiCommand`)
So you must run the Web UI project that provides that service (and serves the browser UI).
Run (example):
```bash
# In the Web UI repository (path/to/nao_server_web/server_web)
./run_app.sh```
### LED action server (optional but recommended for `ui_stub`)
`ui_stub` triggers the LED action **best-effort**:
- `leds_play` (`nao_led_interfaces/action/LedsPlay`)
Run the LED server (from the LED repository / workspace overlay):
```bash
ros2 run nao_led_server led_action_server
```
If the LED server is not running, `ui_stub` continues after a small retry loop
(see `led_retries` parameter).
### Pose action server (required for `nao_pos_bridge.py`)
`nao_pos_bridge.py` sends goals to the pose action:
- `/nao_pos_action`
Run the pose server:
```bash
ros2 run nao_pos_server nao_pos_action_server
```
### Camera publisher (required for `opencv_eval_node.py`)
`opencv_eval_node.py` subscribes to a camera topic (default: `/image_raw`).
You must have *some* camera publisher running (simulation or real robot).
Quick check:
```bash
ros2 topic list | grep image
ros2 topic echo /image_raw --once
```
If your camera topic is not `/image_raw`, launch the node with:
```bash
ros2 run nao_hri_demo_nodes opencv_eval_node.py --ros-args \
-p image_topic:=/your/image/topic
```
### Speech pipeline (required for `speak_bridge.py`)
`speak_bridge.py` bridges `/hri/speak_cmd` into your speech subsystem. In the setup used
for this project, it depends on:
1) **Playback** (`sound_play`), and
2) **TTS generation** (`gtts_service.py`) which produces audio from text.
Run playback:
```bash
python3 path/to/audio_common/sound_play/scripts/soundplay_node.py --ros-args \
-p device:="\"\""
```
Run TTS service:
```bash
python3 path/to/nao_hri/hni_py/hni_py/gtts_service.py
```#### Google Cloud Text-to-Speech (token/credentials)
If your `gtts_service.py` is configured to use **Google Cloud Text-to-Speech**, you need
Google Cloud credentials (a service account key) available at runtime.
- Official docs (client libraries / create audio):
https://docs.cloud.google.com/text-to-speech/docs/create-audio-text-client-libraries
Typical workflow:
1) Create a Google Cloud project and enable the **Text-to-Speech API**.
2) Create a **service account** with permissions to call TTS.
3) Download a **JSON key** for that service account.
4) Export `GOOGLE_APPLICATION_CREDENTIALS` pointing to the JSON key file.
Example:
```bash
export GOOGLE_APPLICATION_CREDENTIALS="path/to/service-account-key.json"
```
Google's auth docs explain this environment variable approach for service accounts:
https://docs.cloud.google.com/run/docs/authenticating/service-to-service
> Keep the key file out of Git. Treat it as a secret.
#### MediaPipe (pose estimation dependency)
If your evaluation node uses **MediaPipe** (e.g., for human pose estimation), install it
in the same Python environment where you run the evaluator.
- MediaPipe documentation: https://developers.google.com/mediapipe
- MediaPipe Pose solution:
https://developers.google.com/mediapipe/solutions/vision/pose_landmarker
Install (Python):
```bash
pip install mediapipe
```
> Note: MediaPipe wheels can be platform-specific; pin versions if needed.
---
## Nodes and scripts
### `ui_stub`
**Role:** bridge between the BT topic interface and the Web UI server (+ optional LEDs).
- Subscribes: `/hri/ui_cmd` (`std_msgs/String`, JSON payload)
- Publishes: `/hri/ui_result` (`std_msgs/String`, e.g. `running|success|failure`)
- Calls service: `/robot/ui/command` (`server_web_interfaces/srv/UiCommand`)
- Triggers LEDs (best-effort): action `leds_play`
(`nao_led_interfaces/action/LedsPlay`), by loading YAML profiles.
Typical command payloads (published to `/hri/ui_cmd`):
- `{ "state": "speak", "hold": 2.0 }`
- `{ "state": "video", "hold": 5.0 }`
- `{ "state": "comparison", "hold": 6.0 }`
**Why it exists:** it encapsulates retries/timeouts so the BT does not block forever if the
Web UI or LED server is unavailable.
Run:```bash
ros2 run nao_hri_demo_nodes ui_stub
```
Key ROS parameters:
- `profiles_pkg` (default: `nao_led_profiles`)
- `profiles_rel_dir` (default: `config/emotions`)
- `ui_wait_service_s` (default: `0.5`)
- `ui_future_timeout_s` (default: `2.0`)
- `led_wait_action_s` (default: `0.2`)
- `led_retries` (default: `2`)
---
### `nao_pos_bridge.py`
**Role:** converts BT pose commands into a pose execution action.
- Subscribes: `/hri/nao_pos_cmd` (`std_msgs/String`, JSON)
- Publishes: `/hri/nao_pos_result` (`std_msgs/String`)
- Sends action goals: `/nao_pos_action` (provided by a pose server package)
Run:
```bash
ros2 run nao_hri_demo_nodes nao_pos_bridge.py
```
Notes:
- This node is an adapter only; the actual actuation is performed by the pose action server.
---
### `opencv_eval_node.py`
**Role:** local evaluation node that validates an exercise attempt using camera input.
- Subscribes: camera topic (parameter), default `/image_raw`
- Subscribes: `/hri/eval_cmd` (`std_msgs/String`, JSON)
- Publishes: `/hri/eval_result` (`std_msgs/String`)
- Uses: OpenCV + `cv_bridge`
- Loads templates from a directory (`config_dir` parameter)
Install OpenCV + `cv_bridge` in your ROS environment (Ubuntu 22.04 / ROS 2 Rolling):
```bash
sudo apt update
sudo apt install -y \
ros-rolling-cv-bridge \
ros-rolling-image-transport \
python3-opencv
```
Quick check:
```bash
python3 -c "import cv2; print('cv2 OK:', cv2.__version__)"
python3 -c "from cv_bridge import CvBridge; print('cv_bridge OK')"
```
Run (example):
```bash
ros2 run nao_hri_demo_nodes opencv_eval_node.py --ros-args \
-p image_topic:=/image_raw \-p config_dir:=src/thirdparty/nao_hri/nao_hri_demo_nodes/config/opencv_templates \
-p timeout_s:=8.0
```
Parameters:
- `image_topic` (string)
- `config_dir` (string)
- `timeout_s` (float)
---
### `speak_bridge.py`
**Role:** bridges BT speech commands to the speech subsystem.
- Subscribes: `/hri/speak_cmd` (`std_msgs/String`, JSON or plain text)
- Publishes: `/hri/speak_result` (`std_msgs/String`)
- Requires companion processes:
- `sound_play` (playback)
- `gtts_service.py` (TTS generation)
Run:
```bash
python3 \
src/thirdparty/nao_hri/nao_hri_demo_nodes/nao_hri_demo_nodes/scripts/speak_bridge.py
```
---
## Interfaces
This package follows a simple convention:
- Command topics: `/hri/<channel>_cmd`
- Result topics: `/hri/<channel>_result`
Payloads are generally JSON strings inside `std_msgs/String` to keep the BT side simple
and language-agnostic.
---
## Dependencies
### Runtime (direct)
- ROS 2 Rolling (`rclcpp`, `rclpy`, `rclcpp_action`)
- `std_msgs`
- `server_web_interfaces` (for `/robot/ui/command`)
- `nao_led_interfaces` + `nao_led_client_utils` (LED action + YAML loader)
- OpenCV + `cv_bridge` (for `opencv_eval_node.py`)
### Runtime (companion stacks)
- Web UI + ROS bridge providing `/robot/ui/command`
- Pose action server providing `/nao_pos_action` (e.g., `nao_pos_server`)
- Camera publisher (simulation or real robot)
- Speech pipeline (`sound_play` + TTS service)
---
## Build
From the workspace root:
```bash
source /opt/ros/rolling/setup.bashcolcon build
source install/setup.bash
```
---
## Run
The commands below assume you already sourced ROS 2 and the workspace overlay:
```bash
source /opt/ros/rolling/setup.bash
source install/setup.bash
```
Then run the nodes you need:
```bash
ros2 run nao_hri_demo_nodes ui_stub
ros2 run nao_hri_demo_nodes nao_pos_bridge.py
ros2 run nao_hri_demo_nodes opencv_eval_node.py --ros-args \
-p image_topic:=/image_raw \
-p config_dir:=src/thirdparty/nao_hri/nao_hri_demo_nodes/config/opencv_templates \
-p timeout_s:=8.0
python3 \
src/thirdparty/nao_hri/nao_hri_demo_nodes/nao_hri_demo_nodes/scripts/speak_bridge.py
```
---
## Configuration
- `config/opencv_templates/`: template images used by `opencv_eval_node.py`
- LED profiles are expected in `nao_led_profiles` (default) under:
- `config/emotions/*.yml`
Keep template/profile directories inside the workspace so you can reference them with
**relative paths** from the workspace root.
---
## Troubleshooting
- If `ros2 run ...` cannot find a node, confirm:
- you built the workspace (`colcon build`)
- you sourced `install/setup.bash`
- If OpenCV evaluation fails:
- verify `cv_bridge` is installed for your ROS distro
- confirm `image_topic` matches your camera publisher
- If `ui_stub` reports service timeouts:
- check that the Web UI ROS bridge provides `/robot/ui/command`
- increase `ui_future_timeout_s` for slower machines
- If speech does nothing:
- confirm `sound_play` is running and using the correct audio device
- confirm your TTS service is running and authenticated (if using Google Cloud)
---
## License
See the repository-level `LICENSE` file.