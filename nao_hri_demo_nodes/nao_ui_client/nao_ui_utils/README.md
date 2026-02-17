# nao_ui_utils
Small ROS 2 **utility package** used by the NAO_HRI workspace to keep the
Web UI / HRI “command format” consistent across nodes.
This package is meant to be built and used on **Ubuntu 22.04**, either:
- natively, or
- inside a Distrobox container (same as the rest of the workspace tooling).
> **Note:** `nao_ui_utils` provides *utilities* (helpers, shared types / small parsing
> helpers). It typically contains **no standalone nodes** to launch.
---
## Contents
- [What is in this package](#what-is-in-this-package)
- [Where it is used](#where-it-is-used)
- [Dependencies](#dependencies)
- [Build](#build)
- [How to use it](#how-to-use-it)
- [Troubleshooting](#troubleshooting)
- [License](#license)
---
## What is in this package
`nao_ui_utils` is a “glue” package that centralizes small pieces of logic that
would otherwise be duplicated across:
- the Behaviour Tree runner and its adapters,
- the Web UI bridge (service/action),
- any node that produces/consumes JSON UI commands.
Typical responsibilities for a utils package in this workspace include:
- common constants (e.g., UI modes / states),
- command schema helpers (e.g., validating/normalizing a JSON payload),
- convenience builders for “UI command” messages.
Because the exact helper list depends on the repository revision, treat this
package as **an internal dependency**: other packages import it, but you normally
don’t run anything from it directly.
---
## Where it is used
In the NAO_HRI workspace, the utilities from this package are consumed by nodes
that must agree on a shared UI command schema.
Common consumers in this repository/workspace:
- `nao_hri_demo_nodes` (e.g., `ui_stub`, `speak_bridge`, evaluation nodes)
- `nao_bt_controller` (BT runner that emits `/hri/*_cmd` topics)
External (separate repository) but commonly used together:
- Web UI server / ROS 2 bridge providing `/robot/ui/command` (service) and
`/robot/ui/command_action` (action)
---
## Dependencies
### Runtime
- ROS 2 (Rolling in the workspace setup).
- Standard Python/C++ runtime dependencies depending on the implementation
(typically `rclpy` or `rclcpp`, plus basic stdlib).
> If `nao_ui_utils` is a pure library package, it will not add extra system> dependencies beyond what the consumers already need.
---
## Build
From the workspace root (e.g., `nao_ws`):
```bash
source /opt/ros/rolling/setup.bash
colcon build
source install/setup.bash
```
---
## How to use it
You do not “run” `nao_ui_utils`. Instead, build the workspace and make sure
your consumer package depends on it (ament dependency in `package.xml`,
and the appropriate CMake / Python packaging hooks).
Typical usage patterns:
- **Python**: import helpers from `nao_ui_utils` inside bridge nodes.
- **C++**: include headers (if provided) from `nao_ui_utils` in nodes that need
shared UI command logic.
---
## Troubleshooting
- **Import/include not found**
- Rebuild the workspace (`colcon build`) and re-source the overlay
(`source install/setup.bash`).
- Confirm the consumer package declares a dependency on `nao_ui_utils`.
- **Runtime schema mismatch**
- If a node rejects a JSON UI command (unknown mode/state or missing fields),
ensure all producers/consumers are using the same revision of
`nao_ui_utils` and the same expected schema.
---
## License
See the repository-level `LICENSE` file.