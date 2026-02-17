# nao_hri# NAO_HRI — Repository Overview
This repository groups several ROS 2 packages used to build a Human–Robot
Interaction (HRI) demo stack around NAO.
Each package is documented separately; this README provides a **high-level map** and
**where to look next**.
## Contents
- [Packages](#packages)
- [Prerequisites](#prerequisites)
- [Build](#build)
- [Run](#run)
- [Directory layout](#directory-layout)
- [License](#license)
---
## Packages
> Links below point to the package directories 


[`nao_bt_controller`](./nao_bt_controller) | Behaviour Tree (BT) runner/controller
that orchestrates the HRI flow by dispatching high-level commands and waiting for
results.

[`nao_hri_demo_nodes`](./nao_hri_demo_nodes) | Demo ROS 2 nodes and scripts
(stubs/bridges/evaluation helpers) to connect the BT to UI, voice, pose/exercise
evaluation, etc. 

[`nao_led_profiles`](./nao_ui_client/nao_led_profiles) | YAML-based LED “emotion/state” profiles
(patterns, colors, timing) packaged for runtime lookup. | Provide consistent visual
signalling across nodes and hardware configurations. |

[`nao_ui_utils`](./nao_ui_client/nao_ui_utils) | Shared utilities for UI-related message/state
handling (small helpers reused by nodes). | Avoid duplication across UI/bridge
components.


---
## Prerequisites
- ROS 2 (Rolling) installed and sourced.
- `colcon` and standard ROS 2 build tooling.
- A working workspace (example name: `nao_ws`) with this repository placed under
`src/`.
> Any extra dependencies that are package-specific are documented in each package
README.
---
## Build
From the workspace root (e.g., `nao_ws`):
```bash
source /opt/ros/rolling/setup.bash
colcon build
source install/setup.bash
```
---## Run
There is no single “one command” launcher in this top-level README because execution
is typically split across multiple terminals (or tmux).
Start with the package READMEs, in this order:
1. `nao_led_profiles` (profiles must be discoverable at runtime)
2. `nao_hri_demo_nodes` (UI/voice/pose/eval demo nodes)
3. `nao_bt_controller` (BT runner that drives the scenario)
---

> **Platform note (Ubuntu 22.04):**  
> These packages were developed and validated on **Ubuntu 22.04 (Jammy)** and ROS2 rolling. They can be executed either **natively** on Ubuntu 22.04 or inside an **Ubuntu 22.04 Distrobox** container (recommended when isolating dependencies or keeping the host clean).


## License
See the repository license file (or the per-package `package.xml` metadata, if
applicable).