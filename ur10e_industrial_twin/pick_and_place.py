import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    WorkspaceParameters,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
)
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
import threading
import time


# ── Poses ──────────────────────────────────────────────────────────────────────

PRE_PICK_POSE = Pose(
    position=Point(x=0.4, y=0.2, z=0.35),
    orientation=Quaternion(x=0.0, y=0.7071, z=0.0, w=0.7071),
)
PICK_POSE = Pose(
    position=Point(x=0.4, y=0.2, z=0.18),
    orientation=Quaternion(x=0.0, y=0.7071, z=0.0, w=0.7071),
)
PRE_PLACE_POSE = Pose(
    position=Point(x=0.4, y=-0.2, z=0.35),
    orientation=Quaternion(x=0.0, y=0.7071, z=0.0, w=0.7071),
)
PLACE_POSE = Pose(
    position=Point(x=0.4, y=-0.2, z=0.18),
    orientation=Quaternion(x=0.0, y=0.7071, z=0.0, w=0.7071),
)


class PickAndPlaceNode(Node):

    def __init__(self):
        super().__init__("pick_and_place_node")
        self._cb_group = ReentrantCallbackGroup()
        self._action_client = ActionClient(
            self, MoveGroup, "/move_action",
            callback_group=self._cb_group
        )
        self._gripper_pub = self.create_publisher(String, "/gripper_cmd", 10)
        self.get_logger().info("Waiting for MoveGroup action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("MoveGroup ready.")

    # ── Gripper ─────────────────────────────────────────────────────────────────

    def gripper_open(self):
        msg = String()
        msg.data = "open"
        self._gripper_pub.publish(msg)
        self.get_logger().info("Gripper → OPEN")
        time.sleep(0.5)

    def gripper_close(self):
        msg = String()
        msg.data = "close"
        self._gripper_pub.publish(msg)
        self.get_logger().info("Gripper → CLOSE")
        time.sleep(0.5)

    # ── MoveIt goal ─────────────────────────────────────────────────────────────

    def _build_goal(self, pose: Pose) -> MoveGroup.Goal:
        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = "ur_manipulator"
        req.num_planning_attempts = 10
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3

        req.workspace_parameters = WorkspaceParameters()
        req.workspace_parameters.header.frame_id = "base_link"
        req.workspace_parameters.min_corner.x = -1.5
        req.workspace_parameters.min_corner.y = -1.5
        req.workspace_parameters.min_corner.z = -0.5
        req.workspace_parameters.max_corner.x = 1.5
        req.workspace_parameters.max_corner.y = 1.5
        req.workspace_parameters.max_corner.z = 2.0

        pc = PositionConstraint()
        pc.header.frame_id = "base_link"
        pc.link_name = "tool0"
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.01, 0.01, 0.01]
        bv = BoundingVolume()
        bv.primitives.append(box)
        bv.primitive_poses.append(pose)
        pc.constraint_region = bv
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header.frame_id = "base_link"
        oc.link_name = "tool0"
        oc.orientation = pose.orientation
        oc.absolute_x_axis_tolerance = 0.15
        oc.absolute_y_axis_tolerance = 0.15
        oc.absolute_z_axis_tolerance = 0.15
        oc.weight = 1.0

        c = Constraints()
        c.position_constraints.append(pc)
        c.orientation_constraints.append(oc)
        req.goal_constraints.append(c)
        goal.request = req
        return goal

    # ── Move ────────────────────────────────────────────────────────────────────

    def move_to(self, pose: Pose, label: str) -> bool:
        self.get_logger().info(f"Moving to: {label}")
        goal = self._build_goal(pose)

        send_future = self._action_client.send_goal_async(goal)
        while not send_future.done():
            time.sleep(0.05)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"Goal REJECTED: {label}")
            return False

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            time.sleep(0.05)

        error_code = result_future.result().result.error_code.val
        if error_code == 1:
            self.get_logger().info(f"Reached: {label} ✓")
            return True
        else:
            self.get_logger().warn(f"Failed: {label}  (error code {error_code})")
            return False

    # ── Cycle ───────────────────────────────────────────────────────────────────

    def run_cycle(self):
        self.get_logger().info("=" * 48)
        self.get_logger().info("  Starting pick-and-place cycle")
        self.get_logger().info("=" * 48)

        steps = [
            (self.gripper_open,  None),
            (self.move_to,       (PRE_PICK_POSE,  "pre-pick")),
            (self.move_to,       (PICK_POSE,       "pick")),
            (self.gripper_close, None),
            (self.move_to,       (PRE_PICK_POSE,  "lift")),
            (self.move_to,       (PRE_PLACE_POSE, "pre-place")),
            (self.move_to,       (PLACE_POSE,      "place")),
            (self.gripper_open,  None),
            (self.move_to,       (PRE_PLACE_POSE, "retreat")),
        ]

        for fn, args in steps:
            if args is None:
                fn()
            else:
                if not fn(*args):
                    self.get_logger().error("Cycle aborted.")
                    return

        self.get_logger().info("=" * 48)
        self.get_logger().info("  Cycle complete ✓")
        self.get_logger().info("=" * 48)


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run_cycle()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
