import importlib
from typing import Optional
 
from pydantic import BaseModel
from rclpy.action import ActionClient
from rclpy.node import Node


def resolve_action_type(msg_type: str):
    pkg, kind, cls_name = msg_type.split('/')
    if kind != 'action':
        raise ValueError(
            f"{msg_type!r} is not an action type"
            f"'{kind}' interfaces"
        )

    module = importlib.import_module(f'{pkg}.action')
    return getattr(module, cls_name)


class ActionSpec(BaseModel):
    name: str
    msg_type: str


class MotionActions(BaseModel):

    move_group: ActionSpec = ActionSpec(
        name="/move_action",
        msg_type="moveit_msgs/action/MoveGroup",
    )


    gripper: ActionSpec = ActionSpec(
        name="/robotiq_gripper_controller/gripper_cmd",
        msg_type="control_msgs/action/GripperCommand"
    )


class MotionInterfaceList(BaseModel):
    action: MotionActions = MotionActions()


class ActionHandlerClient:

    def __init__(self, node: Node, spec: ActionSpec):
        self.node = node
        self.spec = spec
        self.client = ActionClient(node, resolve_action_type(spec.msg_type), spec.name)
        node.get_logger().info(
            f'Action client on {spec.name} ({spec.msg_type})'
        )

    def wait(self, timeout_sec: float = 10.0) -> bool:

        if self.client.wait_for_server(timeout_sec=timeout_sec):
            return True

        self.node.get_logger().error(
            f'no action server on {self.spec.name} after {timeout_sec}s. '
            f'Check: ros2 action list | grep {self.spec.name.split("/")[1]}'
        )

        return False

    def send(self, goal, timeout_sec: float = 60.0, spin_dt: float = 0.02):
        import time
 
        send_future = self.client.send_goal_async(goal)
        deadline = time.monotonic() + timeout_sec
        while not send_future.done():
            if time.monotonic() > deadline:
                self.node.get_logger().error(
                    f'{self.spec.name}: timed out waiting for goal acceptance')
                return None
            time.sleep(spin_dt)
 
        handle = send_future.result()
        if not handle.accepted:
            self.node.get_logger().error(f'{self.spec.name}: goal rejected')
            return None
 
        result_future = handle.get_result_async()
        while not result_future.done():
            if time.monotonic() > deadline:
                self.node.get_logger().error(
                    f'{self.spec.name}: timed out during execution, cancelling')
                handle.cancel_goal_async()
                return None
            time.sleep(spin_dt)
 
        return result_future.result().result
