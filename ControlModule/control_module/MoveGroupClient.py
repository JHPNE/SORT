from typing import Optional
 
from geometry_msgs.msg import Pose, PoseStamped, Vector3
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, OrientationConstraint, PositionConstraint,
    BoundingVolume, MotionPlanRequest, PlanningOptions, WorkspaceParameters,
)
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
 
from control_module.ControlInterface import ActionHandlerClient, MotionInterfaceList

class MoveGroupClient:
    def __init__(self, node: Node,
                 group_name: str = "manipulator",
                 tool_link: str = "tool_frame",
                 reference_frame: str = "base_link",
                 position_tolerance: float = 0.005,
                 orientation_tolerance: float = 0.05,
                 velocity_scaling: float = 0.05,
                 acceleration_scaling: float = 0.15,
                 planning_time: float = 5.0,
                 planning_attempts: int = 10,
                 pipeline_id: str = "",
                 planner_id: str = ""): 

        self.node = node
        self.group = group_name
        self.tool_link = tool_link
        self.ref_frame = reference_frame
        self.pos_tol = position_tolerance
        self.ori_tol = orientation_tolerance
        self.vel_scale = velocity_scaling
        self.acc_scale = acceleration_scaling
        self.planning_time = planning_time
        self.planning_attempts = planning_attempts
        # Empty strings mean "use whatever move_group has configured as
        # default". Only set these if you know the pipeline and planner are
        # actually loaded on the robot PC, or planning fails with an
        # unhelpful error about an unknown planner.
        self.pipeline_id = pipeline_id
        self.planner_id = planner_id

        specs = MotionInterfaceList()
        self.action = ActionHandlerClient(node, specs.action.move_group)
 
    def wait(self, timeout_sec: float = 15.0) -> bool:
        return self.action.wait(timeout_sec)

    def _constraints(self, pose: PoseStamped) -> Constraints:
        pc = PositionConstraint()
        pc.header.frame_id = pose.header.frame_id
        pc.link_name = self.tool_link
        pc.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)
 
        region = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.pos_tol]
        region.primitives = [sphere]
        # Only the position of this pose is used; the sphere has no orientation.
        centre = Pose()
        centre.position = pose.pose.position
        centre.orientation.w = 1.0
        region.primitive_poses = [centre]
        pc.constraint_region = region
        pc.weight = 1.0
 
        oc = OrientationConstraint()
        oc.header.frame_id = pose.header.frame_id
        oc.link_name = self.tool_link
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = self.ori_tol
        oc.absolute_y_axis_tolerance = self.ori_tol
        oc.absolute_z_axis_tolerance = self.ori_tol
        oc.weight = 1.0
 
        c = Constraints()
        c.name = "pose_goal"
        c.position_constraints = [pc]
        c.orientation_constraints = [oc]
        return c

    def _goal(self, pose: PoseStamped, plan_only: bool) -> MoveGroup.Goal:
        req = MotionPlanRequest()
        req.group_name = self.group
        req.goal_constraints = [self._constraints(pose)]
        req.num_planning_attempts = self.planning_attempts
        req.allowed_planning_time = self.planning_time
        req.max_velocity_scaling_factor = self.vel_scale
        req.max_acceleration_scaling_factor = self.acc_scale
        if self.pipeline_id:
            req.pipeline_id = self.pipeline_id
        if self.planner_id:
            req.planner_id = self.planner_id
 
        # Set explicitly, otherwise move_group logs a warning every call and
        # substitutes a default box that may be smaller than your reach.
        ws = WorkspaceParameters()
        ws.header.frame_id = self.ref_frame
        ws.min_corner = Vector3(x=-1.5, y=-1.5, z=-1.5)
        ws.max_corner = Vector3(x=1.5, y=1.5, z=1.5)
        req.workspace_parameters = ws
 
        opts = PlanningOptions()
        opts.plan_only = plan_only
        opts.planning_scene_diff.is_diff = True
        opts.planning_scene_diff.robot_state.is_diff = True
 
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = opts
        return goal

    def go(self, pose: PoseStamped, plan_only: bool = True,
           label: str = "goal", timeout_sec: float = 60.0) -> bool:
        """Plan to a pose, and execute it unless plan_only.
 
        Always run plan_only=True first on a new pose. It exercises IK,
        collision checking and the planner without moving anything, so a bad
        pose costs you a log line instead of a collision.
        """
        p = pose.pose.position
        self.node.get_logger().info(
            f"{'planning' if plan_only else 'executing'} '{label}' -> "
            f"({p.x:+.3f}, {p.y:+.3f}, {p.z:+.3f}) in {pose.header.frame_id}")
 
        result = self.action.send(self._goal(pose, plan_only),
                                  timeout_sec=timeout_sec)
        if result is None:
            return False
 
        code = result.error_code.val
        if code == 1:                                  # SUCCESS
            n = len(result.planned_trajectory.joint_trajectory.points)
            self.node.get_logger().info(
                f"'{label}' ok, {n} trajectory points")
            return True
 
        self.node.get_logger().error(
            f"'{label}' failed: {describe_error(code)} (code {code})")
        return False

    def describe_error(code: int) -> str:
        """MoveItErrorCodes values worth recognising on sight."""
        return {
            1: "SUCCESS",
            -1: "FAILURE",
            -2: "PLANNING_FAILED",
            -3: "INVALID_MOTION_PLAN",
            -4: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
            -5: "CONTROL_FAILED",
            -6: "UNABLE_TO_AQUIRE_SENSOR_DATA",
            -7: "TIMED_OUT",
            -8: "PREEMPTED",
            -10: "START_STATE_IN_COLLISION",
            -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
            -12: "GOAL_IN_COLLISION",
            -13: "GOAL_VIOLATES_PATH_CONSTRAINTS",
            -14: "GOAL_CONSTRAINTS_VIOLATED",
            -15: "INVALID_GROUP_NAME",
            -17: "INVALID_LINK_NAME",
            -19: "NO_IK_SOLUTION",
            -31: "UNABLE_TO_AQUIRE_SENSOR_DATA",
        }.get(code, "unknown")