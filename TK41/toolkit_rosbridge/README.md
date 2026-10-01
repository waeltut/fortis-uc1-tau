# toolkit_rosbridge

General-purpose ROS 2 topic and service access for software outside ROS.
Target: ROS 2 Humble on Ubuntu 22.04, including an existing Linux devcontainer.
Transport: rosbridge JSON over WebSockets.

This package configures upstream `rosbridge_server` and `rosapi`; it does not
implement a competing bridge protocol. External applications can subscribe,
publish, call services and discover topic/service names. Custom interfaces work
when their ROS packages are built and sourced on the bridge host.

## Add to tk4-dev

Extract the archive and put the complete `toolkit_rosbridge` directory anywhere
under your existing ROS workspace's `src` tree (for example inside your tk4-dev
repository).

In your ROS container/host:

```bash
source /opt/ros/humble/setup.bash
sudo apt-get update
sudo apt-get install -y ros-humble-rosbridge-suite python3-yaml python3-colcon-common-extensions
cd ~/ros2_ws
colcon build --symlink-install --packages-select toolkit_rosbridge
source install/setup.bash
ros2 launch toolkit_rosbridge bridge.launch.py
```

If your container runs as root, omit `sudo`. Add `ros-humble-rosbridge-suite`
and `python3-yaml` to the existing Dockerfile's apt installation list so they
survive a rebuild. Alternatively, install manifest dependencies with `rosdep`
using your workspace's normal workflow.

The endpoint is `ws://<ROS_COMPUTER_IP>:9091`. On the same computer use
`ws://localhost:9091`. `0.0.0.0` is a listen address, not a client destination.
Port 9091 avoids the usual rosbridge port 9090; it is not automatically reserved.

```bash
# Change port or bind to one interface
ros2 launch toolkit_rosbridge bridge.launch.py port:=9092 address:=192.168.1.10
# Bind locally only
ros2 launch toolkit_rosbridge bridge.launch.py address:=127.0.0.1
# Use a configuration file (absolute path)
ros2 launch toolkit_rosbridge bridge.launch.py config:=/absolute/path/bridge.yaml
```

Defaults live in `config/bridge.yaml`. Explicit address/port launch arguments
override that file. This is a flat toolkit configuration, not a ROS parameter
YAML. ROS_DOMAIN_ID is inherited from the shell/container, never hardcoded.
Use the same domain and DDS configuration as the toolkit's other ROS nodes.
For example, if those nodes use domain 20, set `export ROS_DOMAIN_ID=20` before
launching. The external WebSocket client does not need a ROS domain setting.

## Verify with the isolated demo

Start the bridge and demo together:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch toolkit_rosbridge bridge.launch.py demo:=true
```

The demo is off by default. It publishes a string once per second, echoes input
strings through a ROS subscription/publisher, and offers a Trigger service.
Wait until it reports ready and ROS discovery has completed.

On any computer with Python 3.10 or newer, copy `examples/` and run:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r examples/requirements.txt
python examples/client.py --url ws://ROS_COMPUTER_IP:9091 smoke
```

Replace `ROS_COMPUTER_IP` with the real host address. On Windows use `py -m venv
.venv` and `.venv\Scripts\Activate.ps1` in PowerShell. The client requires no ROS.
Expected: three PASS lines covering subscription, publish/echo, and service
request/response (including rosapi discovery).

Client commands (global options precede the subcommand):

```bash
python examples/client.py --url ws://ROS_COMPUTER_IP:9091 topics
python examples/client.py --url ws://ROS_COMPUTER_IP:9091 services
python examples/client.py --url ws://ROS_COMPUTER_IP:9091 subscribe /bridge_demo/status std_msgs/msg/String --count 3
python examples/client.py --url ws://ROS_COMPUTER_IP:9091 publish /bridge_demo/input std_msgs/msg/String '{"data":"Hello from the twin"}'
python examples/client.py --url ws://ROS_COMPUTER_IP:9091 call /bridge_demo/ping '{}'
```

To see the published message on ROS, start this BEFORE the publish command:

```bash
ros2 topic echo /bridge_demo/input std_msgs/msg/String
```

For real interfaces, replace the name/type/payload with your topic or service.
Use `ros2 topic list -t`, `ros2 service list -t` and `ros2 interface show TYPE`
to learn the schema. ROS-side consumers/producers remain ordinary ROS nodes.

## Docker/network setup

Run the bridge inside the same ROS environment as your toolkit. On Linux,
existing host networking makes the WebSocket port reachable on the host IP and
can simplify DDS communication. Keep your current working DDS setup.

If the container uses Docker bridge networking, publish the TCP port when
creating the container, for example `-p 9091:9091` or Compose `ports: ["9091:9091"]`.
This exposes WebSockets only; it does not fix ROS discovery between containers.
The bridge and robot nodes must already be able to discover each other.
Allow incoming TCP 9091 from the team's machines through the host firewall.
The client uses the host's reachable address, not a private container address.

## Access scope

The supplied development configuration listens on all IPv4 interfaces and
allows all topics/services/rosapi operations. It provides no user authentication
or TLS. Use on your trusted project network. Remote deployments should use an
authenticated VPN or gateway; an HTTPS browser application normally needs WSS.
This package does not configure a public endpoint, authentication or certificates.

Legacy `topics_glob`, `services_glob` and `params_glob` strings are forwarded to
the installed upstream launch file. Empty strings mean unrestricted access.
Glob behaviour can vary by rosbridge version: use upstream documentation for
your installed version and verify allowed AND disallowed operations before
relying on filtering. `topics_glob` is not separate read/write authorization.
Restricting services can also block `/rosapi/*` discovery calls. Hiding a topic
from discovery alone is not access control. No allowlist is enabled by default.

## Files and handoff

- `launch/bridge.launch.py`: configuration and upstream launch inclusion.
- `config/bridge.yaml`: endpoint and upstream filtering settings.
- `scripts/demo_node.py`: optional ROS-side test endpoints.
- `examples/client.py`: ROS-free CLI and minimal Python client.
- `docs/INTEGRATION.md`: hand this to the digital-twin team.
- `docs/TROUBLESHOOTING.md`: network, ROS, QoS and schema checks.
- `tests/test_client.py`: ROS-free WebSocket client tests.
- `docs/VALIDATION.md`: completed checks and required live acceptance tests.

Read the integration guide before connecting control topics. The bridge is a
transport; it does not validate application permissions, execute trajectories,
confirm motion completion or implement a command watchdog.

## Sources

- [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite)
- [Protocol](https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/ROSBRIDGE_PROTOCOL.md)
- [Upstream launch](https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/rosbridge_server/launch/rosbridge_websocket_launch.xml)
- [Python websockets](https://websockets.readthedocs.io/)

Use the rosbridge release supplied for your ROS distribution. The latest online
protocol also documents features that may not exist in your Humble release;
this example uses the basic JSON topic/service operations only.
