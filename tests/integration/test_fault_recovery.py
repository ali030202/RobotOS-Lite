"""Run after starting demo.launch.py enable_fault_injection:=true in another terminal."""

import time

import pytest


@pytest.mark.integration
def test_sensor_dropout_aborts_and_requires_reset():
    rclpy = pytest.importorskip('rclpy')
    from rclpy.action import ActionClient
    from action_msgs.msg import GoalStatus
    from robotos_interfaces.action import MoveJoints
    from robotos_interfaces.msg import RobotStatus
    from robotos_interfaces.srv import ResetFault, SetSensorDropout
    from sensor_msgs.msg import JointState

    rclpy.init()
    node = rclpy.create_node('robotos_fault_test')
    statuses = []
    joints = []
    node.create_subscription(RobotStatus, '/robot/status', statuses.append, 10)
    node.create_subscription(JointState, '/robot/joint_states', joints.append, 10)
    action = ActionClient(node, MoveJoints, '/robot/move_joints')
    dropout = node.create_client(SetSensorDropout, '/sim/set_sensor_dropout')
    reset = node.create_client(ResetFault, '/robot/reset_fault')

    def until(predicate, seconds=3.0):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def service(client, request):
        future = client.call_async(request)
        assert until(future.done), 'ROS service timed out'
        return future.result()

    try:
        if not (action.wait_for_server(timeout_sec=2.0) and
                dropout.wait_for_service(timeout_sec=2.0) and
                reset.wait_for_service(timeout_sec=2.0)):
            pytest.skip('demo launch with fault injection is not running')
        assert until(lambda: statuses and statuses[-1].mode == 'IDLE' and joints, 4.0)
        goal = MoveJoints.Goal()
        goal.target = [-1.0 if value > 0 else 1.0 for value in joints[-1].position]
        goal.max_velocity = 0.2
        accepted = action.send_goal_async(goal)
        assert until(accepted.done)
        handle = accepted.result()
        assert handle.accepted
        result_future = handle.get_result_async()
        assert until(lambda: statuses and statuses[-1].mode == 'RUNNING' and joints, 2.0)
        before = list(joints[-1].position)
        reply = service(dropout, SetSensorDropout.Request(enabled=True))
        assert reply.accepted
        assert until(lambda: statuses and statuses[-1].mode == 'FAULT', 1.8)
        assert statuses[-1].fault_code == 'SENSOR_TIMEOUT'
        assert until(result_future.done, 1.0)
        assert result_future.result().status == GoalStatus.STATUS_ABORTED
        assert result_future.result().result.code == 'SENSOR_TIMEOUT'
        count_before_recovery = len(joints)
        assert service(dropout, SetSensorDropout.Request(enabled=False)).accepted
        assert until(lambda: len(joints) >= count_before_recovery + 2, 2.0)
        after = list(joints[-1].position)
        assert max(abs(a - b) for a, b in zip(after, before)) <= 0.16
        assert statuses[-1].mode == 'FAULT'
        assert service(reset, ResetFault.Request()).accepted
        assert until(lambda: statuses and statuses[-1].mode == 'IDLE')
    finally:
        if dropout.service_is_ready():
            try:
                service(dropout, SetSensorDropout.Request(enabled=False))
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()
