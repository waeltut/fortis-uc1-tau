# External application integration

## Connection contract

Obtain the ROS host address and port from the toolkit team. Default endpoint:
`ws://<host>:9091/`. Open one persistent WebSocket and send UTF-8 JSON objects
as text messages, one operation per WebSocket message. No HTTP REST endpoint,
ROS installation or DDS setup is required on the external client.

This is the standard rosbridge protocol, not a toolkit-specific envelope.
The team may use a native WebSocket library, roslibjs (JavaScript), or roslibpy
(Python). The included Python client uses raw protocol messages to demonstrate
the contract independently of ROS client libraries.

## Subscribe

Send:

```json
{"op":"subscribe","id":"sub-status","topic":"/bridge_demo/status","type":"std_msgs/msg/String","throttle_rate":100,"queue_length":1}
```

Receive repeatedly (extra fields such as id may also be present):

```json
{"op":"publish","topic":"/bridge_demo/status","msg":{"data":"toolkit bridge demo: 1"}}
```

`throttle_rate` is a minimum forwarding interval in milliseconds, not a request
for the ROS publisher to generate data faster. `queue_length: 1` favours recent
state when throttled; use different buffering for events that must be retained.
A subscription alone does not create data or guarantee a historical sample.

Stop:

```json
{"op":"unsubscribe","id":"sub-status","topic":"/bridge_demo/status"}
```

## Publish

First create the ROS publisher:

```json
{"op":"advertise","id":"pub-input","topic":"/bridge_demo/input","type":"std_msgs/msg/String","queue_size":10,"latch":false}
```

Then send messages over the same connection:

```json
{"op":"publish","id":"message-1","topic":"/bridge_demo/input","msg":{"data":"Hello from the twin"}}
```

When finished:

```json
{"op":"unadvertise","topic":"/bridge_demo/input"}
```

ROS discovery takes time; immediate one-shot publications may miss subscribers.
The CLI includes a configurable initial discovery delay, which is not a delivery
guarantee. The smoke test retries only its harmless demo echo. A production
command interface should define its own acknowledgement and request identifier.
Do not blindly retry robot commands. An operation id is for protocol correlation,
not an application-level duplicate suppression guarantee.

## Call a service

With the demo enabled:

```json
{"op":"call_service","id":"ping-1","service":"/bridge_demo/ping","args":{}}
```

Successful protocol response:

```json
{"op":"service_response","id":"ping-1","service":"/bridge_demo/ping","values":{"success":true,"message":"pong"},"result":true}
```

Match responses by id; responses can arrive among topic messages. The outer
`result` describes the service operation at protocol level. The contents of
`values` are service-specific: a Trigger response also has `success`, and that
can be false even if the outer result is true. Set a client timeout. A timeout
means the outcome is unknown; it does not cancel a ROS service or prove that
the requested work was not performed.

The example client prints service values verbatim; application-level success
interpretation belongs to the caller. Its smoke command explicitly checks the
demo Trigger's success and pong response.

## Discover interfaces

These are themselves service calls through rosapi:

```json
{"op":"call_service","id":"topics-1","service":"/rosapi/topics","args":{}}
```

```json
{"op":"call_service","id":"services-1","service":"/rosapi/services","args":{}}
```

`/rosapi/topics` returns parallel `topics` and `types` arrays in `values`.
Discovery reflects the running graph and may take time to settle. Ask the ROS
team for the precise definitions of custom types; they can run:

```bash
ros2 topic list -t
ros2 service list -t
ros2 interface show std_msgs/msg/String
ros2 interface show std_srvs/srv/Trigger
```

## Data conventions

Preserve the ROS field structure and use full ROS 2 type names such as
`geometry_msgs/msg/PoseStamped`. Nested messages become objects and arrays
remain arrays. Do not wrap a JSON object in a String unless the topic actually
uses `std_msgs/msg/String` as an agreed JSON envelope. Custom interface packages
must be installed and sourced on the bridge host; the twin needs their schema.

Confirm units and frame conventions with the topic owner. For standard geometry
interfaces, use metres/radians and ROS frame definitions; frame_id identifies
the reference frame. Quaternion fields are x, y, z, w. Do not assume the twin's
axis directions match ROS or that all application-defined numeric fields use SI.
ROS stamps use integer seconds and nanoseconds; ROS simulation time may differ
from wall-clock time. Preserve timestamps and distinguish source time from
client receive time. Handle large int64 values carefully in JavaScript.

For visualisation, joint positions alone are not a robot model. Obtain the
URDF/meshes and frame mapping separately if needed. `/tf` and `/tf_static`
need correct QoS and timestamp handling, particularly for late joiners. Images
and point clouds can be expensive over JSON; measure bandwidth and latency
before selecting this transport for full-rate bulk sensor data.

## Connection lifecycle and failures

- Dispatch incoming messages by `op`, then topic or service id. Multiple
  subscriptions share the same WebSocket.
- Log `status` messages; `level: "error"` is a failed protocol operation.
- Track connection health and age of received state; show stale/disconnected
  data rather than presenting the last value as current.
- On reconnect, re-advertise publishers and re-subscribe. Use bounded backoff.
  Do not replay previous publications/service calls automatically.
- Implement local command timeouts/watchdogs in ROS if streaming motion
  commands are permitted. WebSocket disconnect does not itself stop a robot.

The bundled CLI deliberately exits on connection failure, protocol error or a
response/inactivity timeout (default 10 s; change with `--timeout`). It does not
perform automatic reconnection or buffer commands. Its Bridge class has one
sequential receiver; a full twin should use a single background receive loop
that routes messages to subscriptions and pending service futures.

## What the teams must agree on

For each actual interface, record name, ROS type/schema, direction, units/frame,
update rate, stale timeout and application acknowledgement/error behaviour.
The bridge is generic and requires no source edits when those interfaces change.
Services provide request/response; long-running cancellable tasks often need
ROS actions or an application-specific service/topic design. This package makes
no cross-version promise of ROS action support.
