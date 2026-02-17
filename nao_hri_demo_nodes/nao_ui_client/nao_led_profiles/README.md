# nao_led_profiles
A small ROS 2 **configuration package** that ships LED “emotion/interaction”
profiles as YAML files.
These profiles are consumed by LED clients (e.g., `ui_stub` from
[`nao_hri_demo_nodes`](../nao_hri_demo_nodes)) to trigger the `leds_play` action
(from the **nao_led** stack) with consistent parameters.
> This package contains **no nodes** to run. It is installed/built so other nodes
can
> resolve the YAML files via the ROS package share directory.
---
## Contents
- [What is in this package](#what-is-in-this-package)
- [Directory layout](#directory-layout)
- [Dependencies](#dependencies)
- [Build](#build)
- [How it is used](#how-it-is-used)
- [Profile format](#profile-format)
- [Adding a new profile](#adding-a-new-profile)
- [Troubleshooting](#troubleshooting)
- [License](#license)
---
## What is in this package
`nao_led_profiles` provides a set of **named YAML profiles** (typically one per
interaction
state such as `neutral`, `speak`, `video`, etc.). Each profile encodes the
parameters needed
to send a `LedsPlay` goal.
In this workspace, the main consumer is:
- [`nao_hri_demo_nodes/ui_stub`](../nao_hri_demo_nodes) (best-effort LED signalling
during UI state changes)
The LED action server itself lives in the separate repository:
- [`nao_led_packages`](https://github.com/Ruben249/nao_led_packages)
---
## Directory layout
Typical layout (relative to this package root):
- `config/`
- `emotions/`
- `neutral.yml`
- `speak.yml`
- `video.yml`
- `comparison.yml`
- `nao_pos.yml`
- *(more profiles as needed)*
The exact filenames used depend on how the consumer maps states to profiles.
---## Dependencies
### Runtime
- ROS 2 (to resolve package share paths once the workspace is built/installed).
- A consumer that loads these YAML files (e.g., `nao_led_client_utils`).
### LED stack (external, required by consumers)
To actually **play** LEDs you also need the LED action server + interfaces:
- `nao_led_interfaces` (defines the `LedsPlay` action)
- `nao_led_server` (provides the `led_action_server` executable)
Both are part of: [`nao_led`](https://github.com/Ruben249/nao_led)
---
## Build
From the workspace root:
```bash
source /opt/ros/rolling/setup.bash
colcon build
source install/setup.bash
```
Building installs the YAML files into the package share directory, so consumers can
load profiles by package name + relative path.
---
## How it is used
### Example: `ui_stub` (from `nao_hri_demo_nodes`)
`ui_stub` loads a profile from:
- `profiles_pkg` (default: `nao_led_profiles`)
- `profiles_rel_dir` (default: `config/emotions`)
and then picks a profile name based on the UI “state” (e.g., `speak` → `speak.yml`).
You can override where profiles are loaded from:
```bash
ros2 run nao_hri_demo_nodes ui_stub --ros-args \
-p profiles_pkg:=nao_led_profiles \
-p profiles_rel_dir:=config/emotions
```
> Note: the actual LED action server must be running (e.g., `ros2 run nao_led_server
led_action_server`)
> for LED goals to take effect.
---
## Profile format
Profiles must match what the LED client expects to send as a `LedsPlay` goal.
In this workspace, the conversion function used by the client maps YAML into the
following goal fields:
- `leds`
- `mode`
- `frequency`- `duration`
- `colors`
- `intensities`
Keep profiles minimal and consistent. If you change the schema, update the consumer
(`nao_led_client_utils` loader and/or the consuming node).
---
## Adding a new profile
1. Create a new YAML file under `config/emotions/` (e.g., `encouragement.yml`).
2. Ensure its fields map cleanly to the `LedsPlay` goal fields listed above.
3. Rebuild the workspace:
```bash
colcon build
source install/setup.bash
```
4. Update the consumer mapping (e.g., in `ui_stub`) if you want the new profile to
be
selected automatically for a given state.
---
## Troubleshooting
- **Profile not found / cannot resolve package share path**
- Confirm the workspace was built and you sourced `install/setup.bash`.
- Confirm the parameter values (`profiles_pkg`, `profiles_rel_dir`) point to the
correct folder.
- **LEDs do not react**
- Confirm the LED action server is running.
- Confirm the action name matches what the consumer uses (e.g., `leds_play`).
---
## License
See the repository-level `LICENSE` file.