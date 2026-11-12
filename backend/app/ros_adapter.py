"""Small asynchronous boundary between FastAPI and a local ROS 2 graph.

Imports of ROS packages are delayed so API tests can use a fake adapter on a
machine without ROS. Runtime requires Jazzy and sourced workspace interfaces.
"""

import asyncio
import threading
import time
from datetime import datetime, timezone


class AdapterError(Exception):
    def __init__(self, code: str, status: int, message: str):
        super().__init__(message)
        self.code, self.status, self.message = code, status, message


class RosAdapter:
    def __init__(self):
        self._lock = threading.Lock()
        self._last_status = None
        self._last_seen = 0.0
        self._last_observed_at = ''
        self._tasks = {}
        self._handles = {}
        self._loop = None

    async def start(self):
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.executors import SingleThreadedExecutor
        from robotos_interfaces.action import MoveJoints
        from robotos_interfaces.msg import RobotStatus
        from robotos_interfaces.srv import SetEStop, ResetFault

        self._loop = asyncio.get_running_loop()
        self._rclpy = rclpy
        self._move_type = MoveJoints
        self._estop_type = SetEStop
        self._reset_type = ResetFault
        rclpy.init()
        self._node = rclpy.create_node('robotos_api')
        self._node.create_subscription(RobotStatus, '/robot/status', self._on_status, 10)
        self._move_client = ActionClient(self._node, MoveJoints, '/robot/move_joints')
        self._estop_client = self._node.create_client(SetEStop, '/robot/set_estop')
        self._reset_client = self._node.create_client(ResetFault, '/robot/reset_fault')
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()

    async def stop(self):
        if not hasattr(self, '_executor'):
            return
        self._executor.shutdown()
        self._thread.join(timeout=2)
        self._node.destroy_node()
        self._rclpy.shutdown()

    def _on_status(self, msg):
        status = {
            'mode': msg.mode, 'fault_code': msg.fault_code,
            'sensor_fresh': msg.sensor_fresh,
            'position': list(msg.position), 'target': list(msg.target),
            'active_task_id': msg.active_task_id, 'progress': round(float(msg.progress), 4),
        }
        with self._lock:
            self._last_status = status
            self._last_seen = time.monotonic()
            self._last_observed_at = datetime.now(timezone.utc).isoformat()

    def status(self):
        with self._lock:
            state = dict(self._last_status) if self._last_status else None
            age = time.monotonic() - self._last_seen if state else None
            observed_at = self._last_observed_at
        if state is None:
            state = {'mode': 'OFFLINE', 'fault_code': 'NONE', 'sensor_fresh': False,
                     'position': [0.0, 0.0], 'target': [0.0, 0.0],
                     'active_task_id': '', 'progress': 0.0}
        if age is None or age > 2.0:
            state['mode'] = 'OFFLINE'
            state['sensor_fresh'] = False
        state['observed_at'] = observed_at
        state['age_ms'] = round(age * 1000) if age is not None else None
        return state

    async def _await_ros(self, future, timeout=1.0):
        bridge = self._loop.create_future()

        def done(source):
            def transfer():
                if bridge.done():
                    return
                try:
                    bridge.set_result(source.result())
                except Exception as exc:
                    bridge.set_exception(exc)
            self._loop.call_soon_threadsafe(transfer)

        future.add_done_callback(done)
        try:
            return await asyncio.wait_for(bridge, timeout)
        except TimeoutError as exc:
            raise AdapterError('ROS_TIMEOUT', 504, 'ROS 控制节点响应超时') from exc
        except Exception as exc:
            raise AdapterError('ROS_ERROR', 503, 'ROS 通信失败') from exc

    async def move(self, target, max_velocity):
        state = self.status()
        if state['mode'] == 'OFFLINE':
            raise AdapterError('ROS_OFFLINE', 503, '控制节点离线')
        if state['mode'] != 'IDLE':
            raise AdapterError('ROBOT_BUSY', 409, '机器人当前不能接受移动任务')
        available = await asyncio.to_thread(self._move_client.wait_for_server, timeout_sec=0.2)
        if not available:
            raise AdapterError('ROS_OFFLINE', 503, '移动服务不可用')
        goal = self._move_type.Goal()
        goal.target = list(target)
        goal.max_velocity = max_velocity
        handle = await self._await_ros(self._move_client.send_goal_async(goal))
        if not handle.accepted:
            raise AdapterError('GOAL_REJECTED', 409, '控制节点拒绝移动任务')
        task_id = bytes(handle.goal_id.uuid).hex()
        with self._lock:
            if len(self._tasks) >= 100:
                oldest = next(iter(self._tasks))
                self._tasks.pop(oldest, None)
                self._handles.pop(oldest, None)
            self._tasks[task_id] = {'task_id': task_id, 'state': 'RUNNING',
                                    'code': None, 'progress': 0.0,
                                    'position': state['position']}
            self._handles[task_id] = handle
        future = handle.get_result_async()
        future.add_done_callback(lambda result: self._loop.call_soon_threadsafe(
            self._complete, task_id, result))
        return {'task_id': task_id, 'state': 'RUNNING'}

    def _complete(self, task_id, future):
        from action_msgs.msg import GoalStatus
        try:
            wrapped = future.result()
            state = {
                GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
                GoalStatus.STATUS_CANCELED: 'CANCELED',
                GoalStatus.STATUS_ABORTED: 'ABORTED',
            }.get(wrapped.status, 'ABORTED')
            code = wrapped.result.code
            position = list(wrapped.result.final_position)
        except Exception:
            state, code, position = 'ABORTED', 'ROS_RESULT_ERROR', None
        with self._lock:
            task = self._tasks.get(task_id)
            if task:
                task.update(state=state, code=code, position=position,
                            progress=1.0 if state == 'SUCCEEDED' else task['progress'])
            self._handles.pop(task_id, None)

    def task(self, task_id):
        with self._lock:
            task = self._tasks.get(task_id)
            result = dict(task) if task else None
        if result is None:
            raise AdapterError('TASK_NOT_FOUND', 404, '未找到任务')
        if result['state'] in {'RUNNING', 'CANCEL_REQUESTED'}:
            status = self.status()
            if status['active_task_id'] == task_id:
                result['progress'] = status['progress']
                result['position'] = status['position']
        return result

    async def cancel(self, task_id):
        with self._lock:
            task = self._tasks.get(task_id)
            handle = self._handles.get(task_id)
        if task is None:
            raise AdapterError('TASK_NOT_FOUND', 404, '未找到任务')
        if handle is None:
            raise AdapterError('TASK_TERMINAL', 409, '任务已经结束')
        response = await self._await_ros(handle.cancel_goal_async())
        if not response.goals_canceling:
            raise AdapterError('CANCEL_REJECTED', 409, '控制节点拒绝取消')
        with self._lock:
            if task_id in self._tasks and self._tasks[task_id]['state'] == 'RUNNING':
                self._tasks[task_id]['state'] = 'CANCEL_REQUESTED'
        return {'task_id': task_id, 'state': 'CANCEL_REQUESTED'}

    async def _service(self, client, request):
        available = await asyncio.to_thread(client.wait_for_service, timeout_sec=0.2)
        if not available:
            raise AdapterError('ROS_OFFLINE', 503, '控制服务不可用')
        response = await self._await_ros(client.call_async(request))
        if not response.accepted:
            raise AdapterError(response.code, 409, '控制节点拒绝操作')
        return {'accepted': True, 'code': response.code}

    async def estop(self, engage):
        request = self._estop_type.Request()
        request.engage = engage
        return await self._service(self._estop_client, request)

    async def reset_fault(self):
        return await self._service(self._reset_client, self._reset_type.Request())
