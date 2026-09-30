# Validation record

Date: 30.09.2026.

## Completed in the build environment

- Python syntax compilation passed for launch, demo, client and tests.
- package.xml parsed successfully; bridge.yaml parsed with expected defaults.
- Client CLI help loaded successfully.
- Four unittest tests passed using Python websockets 16.0 and real local
  WebSocket connections to a mock rosbridge protocol peer:
  - subscription, publication/echo and service response flow;
  - protocol errors, failed service responses, timeout and disconnect;
  - unrelated messages do not extend a request deadline;
  - malformed/non-object/non-finite JSON payload rejection.

Reproduce client tests after installing examples/requirements.txt:

```bash
python3 -m unittest discover -s tests -v
```

These tests validate the example client, not ROS or the upstream bridge.
The environment had no ROS installation, so colcon build, launch execution,
DDS discovery, real ROS message conversion, rosapi and Humble compatibility
could not be exercised here. No connection to robot hardware was made.

## Live acceptance on the toolkit host

1. Install dependencies, build the package and source the workspace per README.
2. Launch `ros2 launch toolkit_rosbridge bridge.launch.py demo:=true`.
3. From a separate computer, run the client's `smoke` command and require all
   three PASS lines. This checks the actual ROS publish/subscribe round trip,
   periodic ROS subscription, service response and rosapi discovery.
4. Verify `topics` and `services` include expected toolkit interfaces.
5. Test a representative custom interface and the QoS of intended sensor/state
   topics, especially transient-local static transforms and best-effort sensors.
6. Stop the bridge and verify the consuming application shows a disconnected
   state. After restarting, confirm it restores subscriptions without replaying
   old commands. The bundled CLI exits and is restarted manually.
7. Restart without `demo:=true` for normal toolkit use.

Record the installed upstream version with:

```bash
dpkg-query -W ros-humble-rosbridge-server ros-humble-rosapi
```

If enabling access filters later, verify denied operations as well as allowed
ones against that installed version. Replace the placeholder maintainer email
in package.xml with the toolkit maintainer's address before publishing a release.
