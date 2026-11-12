"""ROS-native Python worker, isolated from the operator's Conda environment."""
import asyncio
import inspect
import json
import os
import signal
import sys

from .ros_adapter import AdapterError, RosAdapter


async def serve(socket_path):
    adapter = RosAdapter()
    await adapter.start()
    stopped = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stopped.set)

    async def handle(reader, writer):
        try:
            line = await asyncio.wait_for(reader.readline(), 3)
            request = json.loads(line)
            method = request.get('method')
            if method == 'snapshot':
                with adapter._lock:
                    task_ids = list(adapter._tasks)
                result = {'status': adapter.status(),
                          'tasks': {key: adapter.task(key) for key in task_ids}}
            elif method in {'move', 'cancel', 'estop', 'reset_fault'}:
                result = getattr(adapter, method)(**request.get('params', {}))
                if inspect.isawaitable(result):
                    result = await result
            else:
                raise AdapterError('INVALID_METHOD', 400, 'Unknown bridge operation')
            response = {'result': result}
        except AdapterError as exc:
            response = {'error': {'code': exc.code, 'status': exc.status, 'message': exc.message}}
        except Exception:
            response = {'error': {'code': 'BRIDGE_ERROR', 'status': 503,
                                  'message': 'ROS bridge operation failed'}}
        try:
            writer.write((json.dumps(response) + '\n').encode())
            await writer.drain()
        finally:
            writer.close()
            await writer.wait_closed()

    server = await asyncio.start_unix_server(handle, path=socket_path, limit=65536)
    os.chmod(socket_path, 0o600)
    try:
        async with server:
            await stopped.wait()
    finally:
        await adapter.stop()


if __name__ == '__main__':
    asyncio.run(serve(sys.argv[1]))
