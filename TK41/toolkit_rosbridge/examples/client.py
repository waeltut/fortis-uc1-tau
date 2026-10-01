#!/usr/bin/env python3

# Check for correct websockets version! 
# I ran into this error and don't want others to be confused
import sys
from importlib.metadata import PackageNotFoundError, version

try:
    installed_version = version("websockets")
    major_version = int(installed_version.split(".")[0])
except PackageNotFoundError:
    sys.exit(
        "ERROR: websockets is not installed.\n"
        "Install the client's requirements.txt in its Python environment."
    )

if not 15 <= major_version < 17:
    sys.exit(
        f"ERROR: Unsupported websockets version: {installed_version}\n"
        "This client requires websockets>=15,<17.\n"
        "Install the client's requirements.txt in a separate virtual "
        "environment to avoid changing shared dependencies."
    )

# Real program below.


"""Small ROS-free rosbridge JSON client. Python 3.10+; see --help."""
import argparse
import asyncio
import json
import sys
import uuid

from websockets.asyncio.client import connect
from websockets.exceptions import WebSocketException


class Bridge:
    """Sequential example client: only one receive operation at a time."""
    def __init__(self, socket, timeout=10.0):
        self.socket = socket
        self.timeout = timeout

    async def send(self, op, **fields):
        await self.socket.send(json.dumps(dict(op=op, **fields), allow_nan=False))

    async def receive(self, predicate, timeout=None):
        deadline = asyncio.get_running_loop().time() + (self.timeout if timeout is None else timeout)
        while True:
            remaining = deadline - asyncio.get_running_loop().time()
            if remaining <= 0:
                raise TimeoutError('Timed out waiting for rosbridge response')
            packet = json.loads(await asyncio.wait_for(self.socket.recv(), remaining))
            if packet.get('op') == 'status':
                if packet.get('level') == 'error':
                    raise RuntimeError(packet.get('msg', str(packet)))
                print(json.dumps(packet), file=sys.stderr)
            if predicate(packet):
                return packet

    async def call(self, service, args):
        request_id = uuid.uuid4().hex
        await self.send('call_service', id=request_id, service=service, args=args)
        response = await self.receive(lambda p: p.get('op') == 'service_response' and p.get('id') == request_id)
        if response.get('result') is not True:
            raise RuntimeError('Service transport failed: ' + json.dumps(response))
        return response.get('values', {})

    async def subscribe(self, topic, msg_type, throttle_ms=0):
        await self.send('subscribe', topic=topic, type=msg_type,
                        throttle_rate=throttle_ms, queue_length=1)

    async def advertise(self, topic, msg_type):
        await self.send('advertise', topic=topic, type=msg_type, queue_size=10, latch=False)


async def smoke(bridge):
    """Requires demo:=true. Retries ONLY an idempotent demo echo message."""
    topics = await bridge.call('/rosapi/topics', {})
    if '/bridge_demo/status' not in topics.get('topics', []):
        raise RuntimeError('Demo not discovered; launch demo:=true and wait for ROS discovery')
    await bridge.subscribe('/bridge_demo/status', 'std_msgs/msg/String')
    await bridge.receive(lambda p: p.get('topic') == '/bridge_demo/status' and p.get('op') == 'publish')
    print('PASS: ROS -> WebSocket subscription')
    await bridge.subscribe('/bridge_demo/echo', 'std_msgs/msg/String')
    await bridge.advertise('/bridge_demo/input', 'std_msgs/msg/String')
    token = 'bridge-smoke-' + uuid.uuid4().hex
    deadline = asyncio.get_running_loop().time() + bridge.timeout
    while True:
        remaining = deadline - asyncio.get_running_loop().time()
        if remaining <= 0:
            raise TimeoutError('No ROS echo received')
        await bridge.send('publish', topic='/bridge_demo/input', msg={'data': token})
        try:
            await bridge.receive(lambda p: p.get('topic') == '/bridge_demo/echo'
                                 and p.get('msg', {}).get('data') == token, min(1.0, remaining))
            break
        except TimeoutError:
            continue
    print('PASS: WebSocket -> ROS -> WebSocket echo')
    result = await bridge.call('/bridge_demo/ping', {})
    if result.get('success') is not True or result.get('message') != 'pong':
        raise RuntimeError('Unexpected demo response: ' + json.dumps(result))
    print('PASS: service request/response; rosapi discovery also passed')


def object_json(raw):
    try:
        value = json.loads(raw, parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))
    except (ValueError, json.JSONDecodeError) as exc:
        raise argparse.ArgumentTypeError('Expected valid finite JSON') from exc
    if not isinstance(value, dict):
        raise argparse.ArgumentTypeError('Expected a JSON object')
    return value


def positive(raw):
    value = float(raw)
    if not 0 < value < float('inf'):
        raise argparse.ArgumentTypeError('Must be finite and greater than zero')
    return value


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--url', default='ws://localhost:9091')
    p.add_argument('--timeout', type=positive, default=10.0, help='Response timeout in seconds')
    commands = p.add_subparsers(dest='command', required=True)
    commands.add_parser('topics', help='List topics and their types using rosapi')
    commands.add_parser('services', help='List services using rosapi')
    commands.add_parser('smoke', help='Verify all directions against the demo node')
    sub = commands.add_parser('subscribe')
    sub.add_argument('topic')
    sub.add_argument('type')
    sub.add_argument('--throttle-ms', type=int, default=100)
    sub.add_argument('--count', type=int, default=0, help='0: continue until Ctrl+C')
    pub = commands.add_parser('publish')
    pub.add_argument('topic')
    pub.add_argument('type')
    pub.add_argument('message', type=object_json)
    pub.add_argument('--settle', type=positive, default=1.0, help='DDS discovery delay before sending once')
    call = commands.add_parser('call')
    call.add_argument('service')
    call.add_argument('args', nargs='?', type=object_json, default={})
    return p


async def run(args):
    if args.command == 'subscribe' and (args.count < 0 or args.throttle_ms < 0):
        raise ValueError('count and throttle-ms must be non-negative')
    # No automatic reconnect or replay of publications/service requests.
    async with connect(args.url, open_timeout=args.timeout, max_size=10_000_000,
                       ping_interval=20, ping_timeout=20, proxy=None) as socket:
        bridge = Bridge(socket, args.timeout)
        if args.command in ('topics', 'services'):
            print(json.dumps(await bridge.call('/rosapi/' + args.command, {}), indent=2))
        elif args.command == 'call':
            print(json.dumps(await bridge.call(args.service, args.args), indent=2))
        elif args.command == 'smoke':
            await smoke(bridge)
        elif args.command == 'subscribe':
            await bridge.subscribe(args.topic, args.type, args.throttle_ms)
            received = 0
            while args.count == 0 or received < args.count:
                packet = await bridge.receive(lambda p: p.get('op') == 'publish' and p.get('topic') == args.topic)
                print(json.dumps(packet), flush=True)
                received += 1
            await bridge.send('unsubscribe', topic=args.topic)
        elif args.command == 'publish':
            await bridge.advertise(args.topic, args.type)
            await asyncio.sleep(args.settle)
            await bridge.send('publish', topic=args.topic, msg=args.message)
            # Observe server errors if sent; silence is NOT an acknowledgement.
            try:
                await bridge.receive(lambda p: False, timeout=1.0)
            except TimeoutError:
                pass
            await bridge.send('unadvertise', topic=args.topic)
            print('Sent once; topic delivery/application execution is not acknowledged.')


def main():
    args = parser().parse_args()
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        return 130
    except (OSError, TimeoutError, ValueError, RuntimeError, WebSocketException) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
