"""Real local WebSocket tests against a mock protocol peer, NOT ROS integration."""
import argparse
import asyncio
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import unittest

from websockets.asyncio.client import connect
from websockets.asyncio.server import serve
from websockets.exceptions import ConnectionClosed

spec = importlib.util.spec_from_file_location('client', Path(__file__).parents[1] / 'examples/client.py')
client = importlib.util.module_from_spec(spec)
spec.loader.exec_module(client)


class ClientTest(unittest.IsolatedAsyncioTestCase):
    async def test_smoke_roundtrip(self):
        operations = []
        async def peer(ws):
            async for raw in ws:
                p = json.loads(raw)
                operations.append(p)
                if p['op'] == 'call_service':
                    values = ({'topics': ['/bridge_demo/status'], 'types': ['std_msgs/msg/String']}
                              if p['service'] == '/rosapi/topics' else {'success': True, 'message': 'pong'})
                    # Ignore an unrelated response; correlate by id.
                    await ws.send(json.dumps({'op': 'service_response', 'id': 'other', 'result': True}))
                    await ws.send(json.dumps({'op': 'service_response', 'id': p['id'], 'result': True, 'values': values}))
                elif p['op'] == 'subscribe' and p['topic'] == '/bridge_demo/status':
                    await ws.send(json.dumps({'op': 'publish', 'topic': p['topic'], 'msg': {'data': 'alive'}}))
                elif p['op'] == 'publish':
                    await ws.send(json.dumps({'op': 'publish', 'topic': '/bridge_demo/echo', 'msg': p['msg']}))
        async with serve(peer, '127.0.0.1', 0) as server:
            port = server.sockets[0].getsockname()[1]
            args = client.parser().parse_args(['--url', f'ws://127.0.0.1:{port}', 'smoke'])
            with contextlib.redirect_stdout(io.StringIO()) as output:
                await client.run(args)
            self.assertEqual(output.getvalue().count('PASS:'), 3)
        self.assertEqual(sum(p['op'] == 'advertise' for p in operations), 1)
        self.assertEqual(sum(p['op'] == 'call_service' for p in operations), 2)

    async def test_error_timeout_failure_disconnect(self):
        for mode in ('error', 'timeout', 'service_failure', 'disconnect'):
            with self.subTest(mode=mode):
                async def peer(ws):
                    p = json.loads(await ws.recv())
                    if mode == 'error':
                        await ws.send(json.dumps({'op': 'status', 'level': 'error', 'msg': 'invalid service'}))
                    elif mode == 'service_failure':
                        await ws.send(json.dumps({'op': 'service_response', 'id': p['id'], 'result': False}))
                    elif mode == 'disconnect':
                        await ws.close()
                    await ws.wait_closed()
                async with serve(peer, '127.0.0.1', 0) as server:
                    port = server.sockets[0].getsockname()[1]
                    async with connect(f'ws://127.0.0.1:{port}', proxy=None) as ws:
                        expected = (TimeoutError if mode == 'timeout' else
                                    ConnectionClosed if mode == 'disconnect' else RuntimeError)
                        with self.assertRaises(expected):
                            await client.Bridge(ws, timeout=0.1).call('/test', {})

    async def test_unrelated_messages_do_not_extend_deadline(self):
        async def peer(ws):
            try:
                while True:
                    await ws.send(json.dumps({'op': 'publish', 'topic': '/unrelated'}))
                    await asyncio.sleep(0.01)
            except ConnectionClosed:
                pass
        async with serve(peer, '127.0.0.1', 0) as server:
            port = server.sockets[0].getsockname()[1]
            async with connect(f'ws://127.0.0.1:{port}', proxy=None) as ws:
                with self.assertRaises(TimeoutError):
                    await asyncio.wait_for(client.Bridge(ws, timeout=0.1).receive(lambda p: False), 1)

    def test_payload_validation(self):
        for raw in ('[]', 'null', '{"data":NaN}', '{bad}'):
            with self.assertRaises(argparse.ArgumentTypeError):
                client.object_json(raw)
        self.assertEqual(client.object_json('{"data":"hello"}'), {'data': 'hello'})


if __name__ == '__main__':
    unittest.main()
