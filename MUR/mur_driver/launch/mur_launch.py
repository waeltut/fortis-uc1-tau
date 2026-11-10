"""MUR master launch. Install beside the existing mur_driver.launch.py.

Run: ros2 launch mur_driver mur_launch.py
This file also runs its own readiness probe in a separate Python process.
Readiness is the explicit contract below, not proof that every node is healthy.

Linux/Jetson only: the stack runs under a child-subreaper supervisor. Ctrl+C
first requests normal ROS shutdown, then interrupts/terminates/kills remaining
owned descendants (including detached MiR helpers). Shutdown budgets are the
constants near the end of this file. Already-running processes and services
started through external managers (systemd/Docker/SSH) are outside this scope.
A forced kill cannot guarantee application-level cleanup or stop robot motion.
No ROS package dependencies are added; the supervisor uses the standard library.
"""
import os
import sys

# Edit this list to add/remove launches. These start after Nav2 and MoveIt pass checks.
# Each entry: (package, launch filename, launch arguments).
NAV2_LAUNCH = ("slam_config", "nav2_launch.py", {})
EXTRA_LAUNCHES = [
    ("mur_base_selector", "base_selector.launch.py", {}),
]

REQUIRED_TF = [("odom", "base_link"), ("base_link", "mur"),
               ("mur", "left_tcp"), ("mur", "right_tcp")]
REQUIRED_JOINTS = [
    side + joint for side in ("left_", "right_")
    for joint in ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                  "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")
]
IK_SERVICE = "/compute_ik"
FRESHNESS_SECONDS = 3.0


def _stack_launch_description():
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                                IncludeLaunchDescription, LogInfo,
                                RegisterEventHandler)
    from launch.event_handlers import OnProcessExit
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.substitutions import LaunchConfiguration

    def include(package, filename, arguments):
        path = os.path.join(get_package_share_directory(package), "launch", filename)
        if not os.path.isfile(path):
            raise FileNotFoundError(path)
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                        launch_arguments=arguments.items())

    bringup = include("mur_driver", "mur_driver.launch.py", {
        "execute": LaunchConfiguration("execute"),
        "reference_frame": LaunchConfiguration("reference_frame"),
    })
    # Resolve paths before starting hardware so missing files fail immediately.
    nav2 = include(*NAV2_LAUNCH)
    extras = [include(*entry) for entry in EXTRA_LAUNCHES]
    probe = ExecuteProcess(
        cmd=[sys.executable, os.path.abspath(__file__), "--wait-ready",
             "--timeout", LaunchConfiguration("ready_timeout"),
             "--stable", LaunchConfiguration("ready_stable")],
        name="mur_readiness", output="screen",
    )

    nav2_probe = ExecuteProcess(
        cmd=[sys.executable, os.path.abspath(__file__), "--wait-nav2",
             "--timeout", LaunchConfiguration("ready_timeout"),
             "--stable", LaunchConfiguration("ready_stable")],
        name="nav2_moveit_readiness", output="screen",
    )

    def after_nav2(event, context):
        if context.is_shutdown:
            return []
        if event.returncode != 0:
            return [LogInfo(msg="[MUR] Nav2/MoveIt readiness failed. Reachability NOT started. "
                            "Existing nodes remain running for diagnosis.")]
        return [LogInfo(msg="[MUR] Nav2 active and MoveIt responding. Starting extra launches."),
                *extras]

    def after_probe(event, context):
        if context.is_shutdown:
            return []
        if event.returncode != 0:
            return [LogInfo(msg="[MUR] Readiness failed. Extra launches NOT started. "
                            "MUR remains running for diagnosis; see missing checks above.")]
        return [LogInfo(msg="[MUR] MUR ready. Starting Nav2 and waiting for activation."),
                nav2, nav2_probe]

    return LaunchDescription([
        DeclareLaunchArgument("execute", default_value="true"),
        DeclareLaunchArgument("reference_frame", default_value="mur"),
        DeclareLaunchArgument("ready_timeout", default_value="180.0"),
        DeclareLaunchArgument("ready_stable", default_value="5.0"),
        RegisterEventHandler(OnProcessExit(target_action=probe, on_exit=after_probe)),
        RegisterEventHandler(OnProcessExit(target_action=nav2_probe, on_exit=after_nav2)),
        bringup,
        probe,
    ])


def wait_ready():
    import argparse
    import math
    import time
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import JointState, LaserScan
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Polygon
    from moveit_msgs.srv import GetPositionIK
    from tf2_ros import Buffer, TransformListener

    parser = argparse.ArgumentParser()
    parser.add_argument("--wait-ready", action="store_true")
    parser.add_argument("--wait-nav2", action="store_true")
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--stable", type=float, default=5.0)
    args = parser.parse_args()
    if not all(math.isfinite(v) and v > 0 for v in (args.timeout, args.stable)):
        parser.error("timeout and stable must be finite positive seconds")
    if args.wait_nav2:
        return wait_nav2_and_moveit(args.timeout, args.stable)
    rclpy.init(args=[])
    node = Node("mur_startup_readiness")
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    seen = {}
    joints = {}
    scan_frame = [""]

    def odom_callback(msg):
        seen["/odom"] = time.monotonic()

    def scan_callback(msg):
        if msg.ranges and msg.header.frame_id:
            seen["/scan"] = time.monotonic()
            scan_frame[0] = msg.header.frame_id

    def footprint_callback(msg):
        if len(msg.points) >= 3:
            seen["/global_costmap/footprint"] = time.monotonic()

    def joint_callback(msg):
        now = time.monotonic()
        for name, position in zip(msg.name, msg.position):
            if math.isfinite(position):
                joints[name] = now

    subscriptions = [
        node.create_subscription(Odometry, "/odom", odom_callback, qos_profile_sensor_data),
        node.create_subscription(LaserScan, "/scan", scan_callback, qos_profile_sensor_data),
        node.create_subscription(JointState, "/joint_states", joint_callback, qos_profile_sensor_data),
        node.create_subscription(Polygon, "/global_costmap/footprint", footprint_callback,
                                 qos_profile_sensor_data),
    ]
    ik = node.create_client(GetPositionIK, IK_SERVICE)
    start = time.monotonic()
    stable_since = None
    last_log = -float("inf")
    missing = []
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            missing = [topic for topic in ("/odom", "/scan", "/global_costmap/footprint")
                       if now - seen.get(topic, -float("inf")) > FRESHNESS_SECONDS]
            missing += [name for name in REQUIRED_JOINTS
                        if now - joints.get(name, -float("inf")) > FRESHNESS_SECONDS]
            transforms = REQUIRED_TF + ([("odom", scan_frame[0])] if scan_frame[0] else [])
            for parent, child in transforms:
                if not buffer.can_transform(parent, child, Time()):
                    missing.append("TF " + parent + " <- " + child)
                    continue
                # These chains include odometry and must not be stale.
                if parent == "odom":
                    try:
                        stamp = buffer.lookup_transform(parent, child, Time()).header.stamp
                        age = (node.get_clock().now() - Time.from_msg(stamp)).nanoseconds / 1e9
                        if age > FRESHNESS_SECONDS or age < -1.0:
                            missing.append("stale/future TF " + parent + " <- " + child)
                    except Exception:
                        missing.append("TF changed " + parent + " <- " + child)
            if not ik.service_is_ready():
                missing.append(IK_SERVICE)
            if missing:
                stable_since = None
            elif stable_since is None:
                stable_since = now
            elif now - stable_since >= args.stable:
                node.get_logger().info("MUR ready: all checks stable for %.1f s." % args.stable)
                return 0
            if now - start >= args.timeout:
                node.get_logger().error("Readiness timeout: " +
                                        (", ".join(missing) or "stability interval incomplete"))
                return 1
            if now - last_log >= 5.0:
                node.get_logger().info("Waiting: " +
                                       (", ".join(missing) or "checks passed; confirming stability"))
                last_log = now
        return 2
    except (KeyboardInterrupt, ExternalShutdownException):
        return 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()



def wait_nav2_and_moveit(timeout, stable):
    """Poll actual service responses; retry bounded requests without blocking launch."""
    import time
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from lifecycle_msgs.msg import State
    from lifecycle_msgs.srv import GetState
    from rcl_interfaces.msg import ParameterType
    from rcl_interfaces.srv import ListParameters, GetParameters

    rclpy.init(args=[])
    node = Node("mur_nav2_moveit_readiness")
    nav_nodes = ["map_server", "amcl", "controller_server", "planner_server",
                 "bt_navigator", "behavior_server", "waypoint_follower", "velocity_smoother"]
    checks = []
    for name in nav_nodes:
        checks.append(("/" + name + " active",
                       node.create_client(GetState, "/" + name + "/get_state"),
                       GetState.Request(),
                       lambda result: result.current_state.id == State.PRIMARY_STATE_ACTIVE))
    listing = ListParameters.Request()
    listing.depth = 0  # Recursive, matching a full parameter-list request.
    checks.append(("/move_group/list_parameters response",
                   node.create_client(ListParameters, "/move_group/list_parameters"),
                   listing, lambda result: "robot_description" in result.result.names
                   and "robot_description_semantic" in result.result.names))
    get_request = GetParameters.Request()
    get_request.names = ["robot_description", "robot_description_semantic"]
    checks.append(("MoveIt robot descriptions",
                   node.create_client(GetParameters, "/move_group/get_parameters"),
                   get_request, lambda result: len(result.values) == 2 and all(
                       value.type == ParameterType.PARAMETER_STRING and bool(value.string_value)
                       for value in result.values)))
    deadline = time.monotonic() + timeout
    stable_since = None
    last_log = -float("inf")
    missing = [label for label, *_ in checks]
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            missing = []
            pending = []
            for label, client, request, valid in checks:
                if not client.service_is_ready():
                    missing.append(label + " (service unavailable)")
                else:
                    pending.append((label, client, client.call_async(request), valid))
            request_deadline = min(deadline, time.monotonic() + 3.0)
            while (rclpy.ok() and time.monotonic() < request_deadline
                   and any(not future.done() for _, _, future, _ in pending)):
                rclpy.spin_once(node, timeout_sec=0.1)
            if not rclpy.ok():
                return 2
            for label, client, future, valid in pending:
                if not future.done():
                    client.remove_pending_request(future)
                    future.cancel()
                    missing.append(label + " (response timeout)")
                else:
                    try:
                        if not valid(future.result()):
                            missing.append(label + " (not ready)")
                    except Exception as exc:
                        missing.append(label + " (" + str(exc) + ")")
            now = time.monotonic()
            if missing:
                stable_since = None
            elif stable_since is None:
                stable_since = now
            elif now - stable_since >= stable:
                node.get_logger().info("Nav2 active and MoveIt parameter responses verified.")
                return 0
            if now - last_log >= 5.0:
                node.get_logger().info("Waiting: " +
                    (", ".join(missing) or "checks passed; confirming stability"))
                last_log = now
            # Keep processing ROS events between poll rounds, without busy polling.
            next_poll = min(deadline, time.monotonic() + 1.0)
            while rclpy.ok() and time.monotonic() < next_poll:
                rclpy.spin_once(node, timeout_sec=0.1)
        if not rclpy.ok():
            return 2
        node.get_logger().error("Nav2/MoveIt readiness timeout: " +
            (", ".join(missing) or "stability interval incomplete"))
        return 1
    except (KeyboardInterrupt, ExternalShutdownException):
        return 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# Linux/Jetson process ownership boundary. A subreaper adopts orphaned descendants,
# including double-forked helpers and children that create their own session.
# No process-name matching, ROS-domain matching, or system-wide kill commands.
ROS_SHUTDOWN_SECONDS = 12.0
ORPHAN_INTERRUPT_SECONDS = 3.0
TERMINATE_SECONDS = 5.0
KILL_WAIT_SECONDS = 2.0


def generate_launch_description():
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction

    def start_supervisor(context):
        # Preserve launch configuration overrides, including those supplied by a
        # parent IncludeLaunchDescription. The private LaunchService uses these.
        arguments = [str(key) + ":=" + str(value)
                     for key, value in context.launch_configurations.items()]
        cleanup_budget = (ROS_SHUTDOWN_SECONDS + ORPHAN_INTERRUPT_SECONDS
                          + TERMINATE_SECONDS + KILL_WAIT_SECONDS)
        return [ExecuteProcess(
            cmd=[sys.executable, "-u", os.path.abspath(__file__),
                 "--supervise", *arguments],
            name="mur_supervisor", output="screen",
            # Let our scoped cleanup finish before the outer launcher escalates.
            sigterm_timeout=str(cleanup_budget + 5.0),
            sigkill_timeout="5.0",
        )]

    return LaunchDescription([
        DeclareLaunchArgument("execute", default_value="true"),
        DeclareLaunchArgument("reference_frame", default_value="mur"),
        DeclareLaunchArgument("ready_timeout", default_value="180.0"),
        DeclareLaunchArgument("ready_stable", default_value="5.0"),
        OpaqueFunction(function=start_supervisor),
    ])


def _owned_processes():
    """Return pid -> (parent, start ticks, state) for our current descendants."""
    from pathlib import Path
    processes = {}
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            fields = (entry / "stat").read_text().rsplit(")", 1)[1].split()
            processes[int(entry.name)] = (int(fields[1]), int(fields[19]), fields[0])
        except (OSError, ValueError, IndexError):
            continue  # Process exited while /proc was being read.
    owned = {}
    parents = {os.getpid()}
    while parents:
        children = {pid for pid, (parent, _, _) in processes.items()
                    if parent in parents and pid not in owned}
        owned.update((pid, processes[pid]) for pid in children)
        parents = children
    return owned


def _signal_owned(processes, signum):
    """Check process identity again before signaling; prefer race-free pidfds."""
    import signal
    from pathlib import Path
    for pid, (_, start, state) in processes.items():
        if state == "Z":
            continue
        fd = None
        try:
            if hasattr(os, "pidfd_open") and hasattr(signal, "pidfd_send_signal"):
                try:
                    fd = os.pidfd_open(pid)
                except OSError as exc:
                    import errno
                    if exc.errno not in (errno.ENOSYS, errno.EINVAL):
                        raise
            fields = Path("/proc/%d/stat" % pid).read_text().rsplit(")", 1)[1].split()
            if int(fields[19]) != start:
                continue  # PID has been reused.
            if fd is not None:
                signal.pidfd_send_signal(fd, signum)
            else:
                os.kill(pid, signum)
        except (ProcessLookupError, FileNotFoundError):
            pass
        finally:
            if fd is not None:
                os.close(fd)


def _supervise(command):
    """Run one owned process tree; always clean it on shutdown or launcher exit."""
    import ctypes
    import signal
    import subprocess
    import time

    requested = [False]

    def request_shutdown(signum, frame):
        requested[0] = True

    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, request_shutdown)
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(36, 1, 0, 0, 0) != 0:  # PR_SET_CHILD_SUBREAPER
        raise OSError(ctypes.get_errno(), "Cannot establish MUR process ownership")
    if requested[0]:
        return 0
    # Prevent terminal Ctrl+C from bypassing the inner ROS shutdown sequence.
    child = subprocess.Popen(command, start_new_session=True)
    try:
        while child.poll() is None and not requested[0]:
            time.sleep(0.1)
    finally:
        if child.poll() is None:
            print("[MUR] Requesting normal ROS shutdown.", flush=True)
            child.send_signal(signal.SIGINT)
            deadline = time.monotonic() + ROS_SHUTDOWN_SECONDS
            while child.poll() is None and time.monotonic() < deadline:
                time.sleep(0.1)

        def remaining():
            # Popen owns the main child's wait status; reap adopted orphans only
            # after it has collected that status.
            if child.poll() is not None:
                while True:
                    try:
                        pid, _ = os.waitpid(-1, os.WNOHANG)
                    except ChildProcessError:
                        break
                    if pid == 0:
                        break
            return _owned_processes()

        for signum, grace in ((signal.SIGINT, ORPHAN_INTERRUPT_SECONDS),
                              (signal.SIGTERM, TERMINATE_SECONDS),
                              (signal.SIGKILL, KILL_WAIT_SECONDS)):
            processes = remaining()
            if not processes:
                break
            print("[MUR] Remaining owned processes %s: %s" %
                  (sorted(processes), signal.Signals(signum).name), flush=True)
            deadline = time.monotonic() + grace
            signaled = set()
            while processes:
                fresh = {pid: record for pid, record in processes.items()
                         if (pid, record[1]) not in signaled}
                _signal_owned(fresh, signum)
                signaled.update((pid, record[1]) for pid, record in fresh.items())
                if time.monotonic() >= deadline:
                    break
                time.sleep(0.1)
                processes = remaining()
        leftovers = remaining()
        if leftovers:
            print("[MUR] ERROR: processes still present after SIGKILL: %s. "
                  "Check for uninterruptible kernel/I/O waits." % sorted(leftovers),
                  flush=True)
        else:
            print("[MUR] Shutdown complete: no owned processes remain.", flush=True)
    if leftovers:
        return 1
    return 0 if requested[0] else (child.returncode or 0)


if __name__ == "__main__":
    if "--supervise" in sys.argv:
        raise SystemExit(_supervise([
            sys.executable, "-u", os.path.abspath(__file__), "--run-stack",
            *sys.argv[2:]]))
    if "--run-stack" in sys.argv:
        from launch import LaunchDescription, LaunchService
        from launch.actions import SetLaunchConfiguration
        overrides = [SetLaunchConfiguration(*argument.split(":=", 1))
                     for argument in sys.argv[2:]]
        service = LaunchService(argv=sys.argv[2:], noninteractive=True)
        service.include_launch_description(LaunchDescription([
            *overrides, *_stack_launch_description().entities]))
        raise SystemExit(service.run())
    raise SystemExit(wait_ready())
