"""Opt-in API tests against a running ROS + FastAPI deployment."""
import os
import subprocess
import time

import httpx
import pytest

pytestmark = pytest.mark.skipif(os.getenv('ROBOTOS_E2E') != '1', reason='requires live demo')


def wait_for(read, predicate, timeout=12):
    deadline = time.monotonic() + timeout
    value = None
    while time.monotonic() < deadline:
        value = read()
        if predicate(value):
            return value
        time.sleep(0.1)
    raise AssertionError('Timed out waiting for expected state: ' + repr(value))


def dropout(enabled):
    subprocess.run([
        'ros2', 'service', 'call', '/sim/set_sensor_dropout',
        'robotos_interfaces/srv/SetSensorDropout',
        '{enabled: ' + ('true' if enabled else 'false') + '}',
    ], check=True, capture_output=True, text=True, timeout=10)


@pytest.fixture
def api():
    with httpx.Client(base_url='http://127.0.0.1:8000', timeout=5,
                      headers={'X-Operator-Token': os.environ['ROBOTOS_OPERATOR_TOKEN']}) as client:
        wait_for(lambda: client.get('/api/v1/status').json(), lambda s: s['mode'] == 'IDLE')
        yield client


def task(api, task_id):
    return api.get('/api/v1/tasks/' + task_id).json()


def test_live_move(api):
    response = api.post('/api/v1/tasks/move', json={'target': [0.4, -0.4], 'max_velocity': 0.3})
    assert response.status_code == 202, response.text
    final = wait_for(lambda: task(api, response.json()['task_id']), lambda t: t['state'] == 'SUCCEEDED')
    assert max(abs(a - b) for a, b in zip(final['position'], [0.4, -0.4])) <= 0.02


def test_live_cancel(api):
    response = api.post('/api/v1/tasks/move', json={'target': [-1.2, 1.2], 'max_velocity': 0.2})
    assert response.status_code == 202, response.text
    task_id = response.json()['task_id']
    time.sleep(0.25)
    response = api.post('/api/v1/tasks/' + task_id + '/cancel')
    assert response.status_code == 202, response.text
    wait_for(lambda: task(api, task_id), lambda t: t['state'] == 'CANCELED')
    state = wait_for(lambda: api.get('/api/v1/status').json(), lambda s: s['mode'] == 'IDLE')
    assert state['progress'] < 1.0


def test_live_estop(api):
    response = api.post('/api/v1/tasks/move', json={'target': [-1.2, 1.2], 'max_velocity': 0.2})
    assert response.status_code == 202, response.text
    task_id = response.json()['task_id']
    try:
        assert api.post('/api/v1/estop', json={'engage': True}).status_code == 200
        wait_for(lambda: api.get('/api/v1/status').json(), lambda s: s['mode'] == 'ESTOP')
        result = wait_for(lambda: task(api, task_id), lambda t: t['state'] == 'ABORTED')
        assert result['code'] == 'ESTOP'
        assert api.post('/api/v1/tasks/move', json={'target': [0, 0], 'max_velocity': 0.2}).status_code == 409
    finally:
        assert api.post('/api/v1/estop', json={'engage': False}).status_code == 200


def test_live_sensor_fault_recovery(api):
    response = api.post('/api/v1/tasks/move', json={'target': [-1.2, 1.2], 'max_velocity': 0.2})
    assert response.status_code == 202, response.text
    task_id = response.json()['task_id']
    try:
        dropout(True)
        status = wait_for(lambda: api.get('/api/v1/status').json(), lambda s: s['mode'] == 'FAULT', 3)
        assert status['fault_code'] == 'SENSOR_TIMEOUT'
        result = wait_for(lambda: task(api, task_id), lambda t: t['state'] == 'ABORTED')
        assert result['code'] == 'SENSOR_TIMEOUT'
        assert api.post('/api/v1/fault/reset', json={}).status_code == 409
    finally:
        dropout(False)
        wait_for(lambda: api.get('/api/v1/status').json(), lambda s: s['sensor_fresh'])
        assert api.post('/api/v1/fault/reset', json={}).status_code == 200
    wait_for(lambda: api.get('/api/v1/status').json(), lambda s: s['mode'] == 'IDLE')
