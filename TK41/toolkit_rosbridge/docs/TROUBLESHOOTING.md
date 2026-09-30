# Troubleshooting

## Cannot connect

Check the bridge terminal, host IP and port. On the ROS host:

```bash
ss -ltnp | grep ':9091'
ros2 node list
```

If the port is occupied, launch with `port:=9092` and update the client URL.
A bind address of 127.0.0.1 cannot be reached from another machine. Check Docker
port publishing and host firewalls. Connecting to the MiR's bridge at port 9090
is a different endpoint and may expose a different ROS graph.

## WebSocket works but topics are missing

Run these in the bridge's ROS container/environment:

```bash
printenv ROS_DOMAIN_ID
ros2 topic list -t
ros2 service list -t
```

Compare domain, DDS implementation/network configuration and sourced overlays
with the running toolkit. Re-source `~/ros2_ws/install/setup.bash` before launching
the bridge after building custom interface packages. An external client does
not join the DDS domain. Port publishing alone does not solve DDS discovery.

## Topic exists but no messages arrive

Verify the real ROS publisher and subscriber QoS:

```bash
ros2 topic info /your_topic --verbose
ros2 topic echo /your_topic
```

Check type names, filter settings, publisher activity and client timeout.
A rosbridge/ROS subscription must have QoS compatible with the publisher.
Best-effort sensor streams and transient-local state are common special cases.
Support for selecting QoS from a WebSocket request depends on the installed
rosbridge version; do not assume latest online protocol features exist in Humble.
If needed, update to a compatible supported rosbridge release or add a small
ROS-side relay with explicit QoS for that interface. Do not silently change
robot publisher QoS just to make the bridge work.

## Publish says sent but nothing happened

A WebSocket write does not acknowledge ROS delivery or application execution.
Start the ROS subscriber before publishing and allow discovery time. Check the
JSON schema and bridge status logs. Use `smoke` for an actual ROS echo round trip.
The CLI's one-second discovery delay is only a convenience, not a guarantee.

## Services fail or time out

Check the service exists with `ros2 service list -t` and verify its request
schema with `ros2 interface show <package>/srv/<Type>`. Check `services_glob` if
configured. rosapi calls also require permission to their service names.
Increase the client's `--timeout` only when the operation legitimately takes
longer. Do not retry a timed-out non-idempotent service until its outcome is known.
A service server failure may appear as a status error or unsuccessful response.

## Demo smoke fails

Ensure launch used `demo:=true`, wait for discovery and try again. Verify:

```bash
ros2 topic echo /bridge_demo/status --once
ros2 service call /bridge_demo/ping std_srvs/srv/Trigger '{}'
```

Then use the client's subscribe/publish/call commands separately. The demo never
moves hardware. Stop it by restarting the launch without `demo:=true`.

## Package not found or XML launch parser missing

Check the directory is under workspace `src`, build it and source the install
setup in the current shell. Install dependencies from package.xml. `launch_xml`
is needed to include the upstream XML launch file. rosbridge and rosapi must be
installed for the same ROS distribution as the rest of the workspace.

## Browser reports mixed content

An HTTPS page generally cannot open plain `ws://` to your robot. Arrange an
appropriate WSS gateway/certificate and authentication with the deployment team.
Do not disable certificate verification in a production client.
