import os

from fastapi.testclient import TestClient

from app.main import create_app
from app.ros_adapter import AdapterError


class FakeAdapter:
    def __init__(self):
        self.mode = 'IDLE'
        self.items = {}

    def status(self):
        return {
            'mode': self.mode, 'fault_code': 'NONE', 'sensor_fresh': self.mode != 'OFFLINE',
            'position': [0.0, 0.0], 'target': [0.0, 0.0], 'active_task_id': '',
            'progress': 0.0, 'observed_at': '2026-01-01T00:00:00+00:00', 'age_ms': 10,
        }

    async def move(self, target, max_velocity):
        if self.mode == 'OFFLINE':
            raise AdapterError('ROS_OFFLINE', 503, '控制节点离线')
        if self.mode != 'IDLE':
            raise AdapterError('ROBOT_BUSY', 409, '机器人当前不能接受移动任务')
        self.items['task-1'] = {
            'task_id': 'task-1', 'state': 'RUNNING', 'code': None,
            'progress': 0.0, 'position': [0.0, 0.0],
        }
        self.mode = 'RUNNING'
        return {'task_id': 'task-1', 'state': 'RUNNING'}

    def task(self, task_id):
        if task_id not in self.items:
            raise AdapterError('TASK_NOT_FOUND', 404, '未找到任务')
        return self.items[task_id]

    async def cancel(self, task_id):
        self.task(task_id)
        self.items[task_id]['state'] = 'CANCEL_REQUESTED'
        return {'task_id': task_id, 'state': 'CANCEL_REQUESTED'}

    async def estop(self, engage):
        self.mode = 'ESTOP' if engage else 'IDLE'
        return {'accepted': True, 'code': 'OK'}

    async def reset_fault(self):
        self.mode = 'IDLE'
        return {'accepted': True, 'code': 'OK'}


def client():
    os.environ['ROBOTOS_OPERATOR_TOKEN'] = 'test-secret'
    return TestClient(create_app(FakeAdapter()))


def test_auth_and_input_validation():
    with client() as api:
        response = api.post('/api/v1/tasks/move', json={'target': [0.5, -0.5], 'max_velocity': 0.3})
        assert response.status_code == 401
        assert response.json()['error']['code'] == 'UNAUTHORIZED'
        headers = {'X-Operator-Token': 'test-secret'}
        for target in ([2.0, 0.0], [0.1], [0.0, 0.0, 0.0]):
            response = api.post('/api/v1/tasks/move', headers=headers,
                                json={'target': target, 'max_velocity': 0.3})
            assert response.status_code == 422
            assert response.json()['error']['code'] == 'INVALID_INPUT'
        response = api.post('/api/v1/tasks/move', headers=headers,
                            json={'target': [0.5, -0.5], 'max_velocity': 0.3})
        assert response.status_code == 202
        assert response.json()['task_id'] == 'task-1'
        assert api.get('/api/v1/tasks/task-1').json()['state'] == 'RUNNING'
        assert api.post('/api/v1/tasks/move', headers=headers,
                        json={'target': [0, 0], 'max_velocity': 0.2}).status_code == 409
        assert api.post('/api/v1/tasks/task-1/cancel', headers=headers).status_code == 202


def test_status_and_estop():
    with client() as api:
        assert api.get('/api/v1/status').json()['mode'] == 'IDLE'
        headers = {'X-Operator-Token': 'test-secret'}
        assert api.post('/api/v1/estop', headers=headers, json={'engage': True}).json()['accepted']
        assert api.get('/api/v1/status').json()['mode'] == 'ESTOP'
        assert api.post('/api/v1/estop', headers=headers, json={'engage': False}).status_code == 200


def test_websocket_first_frame():
    with client() as api:
        with api.websocket_connect('/ws/v1/status') as socket:
            assert socket.receive_json()['mode'] == 'IDLE'
