# Brian Ta
# Run this inside Isaac Sim Script Editor

import omni.usd
import omni.kit.app
from pxr import UsdPhysics, PhysxSchema
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

ROBOT_PRIM_PATH = "/World/Risk_Sim/firefighter"
TOPIC_NAME = "/isaac_joint_commands"

stage = omni.usd.get_context().get_stage()

if not rclpy.ok():
    rclpy.init()

# Handle mismatched string type between Kilted and Galactic
JOINT_NAME_MAP = {
    "joint1_to_base": "joint1",
    "joint2_to_joint1": "joint2",
    "joint3_to_joint2": "joint3",
    "joint4_to_joint3": "joint4",
    "joint5_to_joint4": "joint5",
    "joint6_to_joint5": "joint6",
    "joint7_to_joint6": "joint7",
}

JOINT_ORDER = [
    "joint1_to_base",
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint7_to_joint6",
]


def get_arm_joint_positions(stage, robot_path):
    found = {}

    for prim in stage.Traverse():
        path = str(prim.GetPath())

        if robot_path not in path:
            continue
        if not prim.IsA(UsdPhysics.RevoluteJoint):
            continue

        isaac_name = path.split("/")[-1]
        if isaac_name not in JOINT_NAME_MAP:
            continue

        pos = None

        state_api = PhysxSchema.JointStateAPI.Get(prim, "angular")
        if state_api:
            pos = state_api.GetPositionAttr().Get()
            if pos is not None:
                # Isaac angular state appears to be in degrees here
                pos = math.radians(float(pos))

        if pos is None:
            drive = UsdPhysics.DriveAPI.Get(prim, "angular")
            if drive:
                target = drive.GetTargetPositionAttr().Get()
                if target is not None:
                    pos = math.radians(float(target))

        if pos is not None:
            found[isaac_name] = float(pos)

    ros_names = []
    ros_positions = []

    for isaac_name in JOINT_ORDER:
        if isaac_name not in found:
            return None, None
        ros_names.append(JOINT_NAME_MAP[isaac_name])
        ros_positions.append(found[isaac_name])

    return ros_names, ros_positions


class JointStatePublisher(Node):
    def __init__(self):
        super().__init__("firefighter_joint_publisher")
        self.pub = self.create_publisher(JointState, TOPIC_NAME, 10)
        self.last_positions = None
        self.get_logger().info(f"Publishing JointState on {TOPIC_NAME}")

    def publish_tick(self):
        names, positions = get_arm_joint_positions(stage, ROBOT_PRIM_PATH)

        if not names or not positions:
            self.get_logger().warn("Could not find all 7 arm joints")
            return

        if len(names) != 7 or len(positions) != 7:
            self.get_logger().error(
                f"Expected 7 joints, got names={len(names)} positions={len(positions)}"
            )
            return

        if positions != self.last_positions:
            print(
                f"Publishing arm joints (rad): "
                f"{list(zip(names, [round(p, 4) for p in positions]))}"
            )
            self.last_positions = positions.copy()

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.name = names
        msg.position = [float(p) for p in positions]
        msg.velocity = []
        msg.effort = []

        self.pub.publish(msg)


pub_node = JointStatePublisher()

_update_stream = omni.kit.app.get_app().get_update_event_stream()
_frame_counter = {"count": 0}

def _on_update(event):
    _frame_counter["count"] += 1
    if _frame_counter["count"] % 6 == 0:
        pub_node.publish_tick()

_subscription = _update_stream.create_subscription_to_pop(
    _on_update,
    name="joint_state_publish",
)

print(f"Publishing {TOPIC_NAME} on main thread via app update loop.")
print("To stop: _subscription.unsubscribe()")

