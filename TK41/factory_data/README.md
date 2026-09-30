# Factory data ROS 2 service — Humble / Python 3.10

Contains two ready-to-build packages:

- `factory_data_interfaces`: custom `GetFactoryData.srv` (generated Python and C++ bindings).
- `factory_data_bridge`: Python service node that fetches JSON from the separate HTTP server.

## Install and build

Extract this ZIP directly into your ROS workspace's `src` directory. It creates
`src/factory_data_ros2/` containing both packages; colcon discovers them recursively.

```bash
mkdir -p ~/ros2_ws/src
unzip factory_data_ros2.zip -d ~/ros2_ws/src
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/factory_data_ros2 --ignore-src -r -y
colcon build --symlink-install --packages-up-to factory_data_bridge
source install/setup.bash
```

Run these inside your ROS container if you use one. No extra pip packages are
required by the node. The maintainer contact in the manifests is a placeholder;
replace it before publishing the packages.

## Start

Start `python3 server.py` from the separate `factory_data_server` distribution first.
Then, in a sourced ROS terminal:

```bash
ros2 launch factory_data_bridge factory_data.launch.py server_url:=http://127.0.0.1:8000/data
```

For a server on another machine, replace `127.0.0.1` with its reachable IP address.
Inside Docker, loopback means the container itself. Use the server's reachable
address (or appropriate host networking) when the server is outside that container.
If the server runs in Docker with bridge networking, publish its port, e.g. `-p 8000:8000`.

Optional parameters:

```bash
ros2 launch factory_data_bridge factory_data.launch.py \
  server_url:=http://192.168.1.20:8000/data \
  timeout_sec:=120.0 \
  max_response_bytes:=104857600
```

`timeout_sec` is the HTTP socket timeout, not an end-to-end ROS service deadline.
`max_response_bytes` defaults to 50 MiB. Large service responses may also require
DDS configuration appropriate to your network and middleware.

Without launch:

```bash
ros2 run factory_data_bridge factory_data_service --ros-args \
  -p server_url:=http://127.0.0.1:8000/data
```

## Request all data

In another terminal, source ROS and the workspace, then call:

```bash
ros2 service call /get_factory_data factory_data_interfaces/srv/GetFactoryData '{}'
```

Interface:

```srv
---
bool success
string message
string json_data
```

Each request triggers a fresh server scan. `json_data` contains the complete JSON
text; callers parse it with `json.loads(response.json_data)` in Python, or a JSON
library in C++. The service name is `get_factory_data` relative to the node's
namespace; by default it is `/get_factory_data`.

`success=true` means a valid aggregate was retrieved, including partial or empty
results. Inspect `skipped_count` and `errors` inside JSON to determine completeness.
`message` summarises included and skipped files. HTTP failures, invalid JSON,
timeouts and oversized responses return `success=false`, an error message and
empty `json_data`. No stale result is returned. Data is returned to the requesting
client; the node does not publish a topic or automatically save a second local copy.
The HTTP server saves its own `output/factory_data.json`.

The node processes requests one at a time. It is a dedicated data-fetch node,
so a slow HTTP call does not block nodes in other processes.

## Example Python client

See `examples/request_data.py`. After sourcing the workspace:

```bash
python3 ~/ros2_ws/src/factory_data_ros2/examples/request_data.py
```

## Verification

The supplied server/parser and HTTP integration tests were run in a Python
environment. A ROS 2 installation was unavailable during creation, so a real
Humble `colcon` build, generated interface import and DDS service call remain to
be verified on your ROS machine.

ROS interface reference:
https://github.com/ros2/ros2_documentation/blob/humble/source/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.rst
