"""Run FastAPI under Conda Python 3.9 while ROS uses its native Python 3.12."""
import asyncio
import contextlib
import json
import os
from pathlib import Path
import shutil
import tempfile
import threading
import time

from .ros_adapter import AdapterError


class ProcessRosAdapter:
    def __init__(self):
        self._lock = threading.Lock()
        self._state = {'mode': 'OFFLINE', 'fault_code': 'NONE', 'sensor_fresh': False,
                       'position': [0.0, 0.0], 'target': [0.0, 0.0],
                       'active_task_id': '', 'progress': 0.0, 'observed_at': '', 'age_ms': None}
        self._tasks = {}
        self._last_poll = 0.0
        self._process = None
        self._poller = None
        self._directory = None

    async def start(self):
        self._directory = tempfile.mkdtemp(prefix='robotos-bridge-')
        self._socket = str(Path(self._directory) / 'ros.sock')
        env = os.environ.copy()
        backend = str(Path(__file__).resolve().parents[1])
        env['PYTHONPATH'] = backend + os.pathsep + env.get('PYTHONPATH', '')
        self._process = await asyncio.create_subprocess_exec(
            os.environ['ROBOTOS_ROS_PYTHON'], '-m', 'app.ros_worker', self._socket, env=env)
        try:
            for _ in range(100):
                if self._process.returncode is not None:
                    raise RuntimeError('ROS worker exited; check ROS setup and Python compatibility')
                if Path(self._socket).exists():
                    await self._refresh()
                    self._poller = asyncio.create_task(self._poll())
                    return
                await asyncio.sleep(0.1)
            raise RuntimeError('ROS worker startup timed out')
        except BaseException:
            await self.stop()
            raise

    async def stop(self):
        if self._poller:
            self._poller.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._poller
        if self._process and self._process.returncode is None:
            self._process.terminate()
            try:
                await asyncio.wait_for(self._process.wait(), 3)
            except asyncio.TimeoutError:
                self._process.kill()
                await self._process.wait()
        if self._directory:
            shutil.rmtree(self._directory)

    async def _rpc(self, method, **params):
        writer = None
        try:
            reader, writer = await asyncio.wait_for(asyncio.open_unix_connection(self._socket), 1)
            writer.write((json.dumps({'method': method, 'params': params}) + '\n').encode())
            await writer.drain()
            response = json.loads(await asyncio.wait_for(reader.readline(), 3))
        except (OSError, ValueError, asyncio.TimeoutError) as exc:
            raise AdapterError('ROS_BRIDGE_OFFLINE', 503, 'ROS 桥接进程不可用') from exc
        finally:
            if writer:
                writer.close()
                await writer.wait_closed()
        if 'error' in response:
            error = response['error']
            raise AdapterError(error['code'], error['status'], error['message'])
        return response['result']

    async def _refresh(self):
        result = await self._rpc('snapshot')
        with self._lock:
            self._state = result['status']
            self._tasks = result['tasks']
            self._last_poll = time.monotonic()

    async def _poll(self):
        while True:
            try:
                await self._refresh()
            except AdapterError:
                pass
            await asyncio.sleep(0.2)

    def status(self):
        with self._lock:
            state = dict(self._state)
            poll_age = time.monotonic() - self._last_poll
        if poll_age > 2:
            state.update(mode='OFFLINE', sensor_fresh=False)
        if state['age_ms'] is not None:
            state['age_ms'] += round(poll_age * 1000)
        return state

    def task(self, task_id):
        with self._lock:
            task = self._tasks.get(task_id)
        if task is None:
            raise AdapterError('TASK_NOT_FOUND', 404, '未找到任务')
        return dict(task)

    async def move(self, target, max_velocity):
        result = await self._rpc('move', target=target, max_velocity=max_velocity)
        await self._refresh()
        return result

    async def cancel(self, task_id):
        result = await self._rpc('cancel', task_id=task_id)
        await self._refresh()
        return result

    async def estop(self, engage):
        return await self._rpc('estop', engage=engage)

    async def reset_fault(self):
        return await self._rpc('reset_fault')
