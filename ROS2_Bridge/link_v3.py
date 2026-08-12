#!/usr/bin/env python3
"""
Brian Ta

link_v3.py — Physical Arm Controller for MyArm
Runs on Raspberry Pi. Bridges ROS 2 topics to physical hardware via serial.

Topics:
  Subscribes: /isaac_joint_commands  (sensor_msgs/JointState)
  Publishes:  /pi_joint_states       (sensor_msgs/JointState)

QoS: RELIABLE / VOLATILE on both publisher and subscriber
     to ensure cross-version compatibility between Galactic and Kilted.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from pymycobot.myarm import MyArm
import math
import time


# Explicit QoS profile — same on both Pi and dt01
ARM_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST
)


class PhysicalArmController(Node):
    def __init__(self):
        super().__init__('physical_arm_controller')

        # Hardware 
        self.get_logger().info("Connecting to MyArm on /dev/ttyAMA0...")
        try:
            self.mc = MyArm('/dev/ttyAMA0', 115200)
        except Exception as e:
            self.get_logger().fatal(f"Failed to open serial port: {e}")
            raise

        self.medium_speed = 50
        self.time_delay = 0.25
        self.last_command_time = time.time()

        # Wait for arm to initialize 
        self.get_logger().info("Waiting for arm to initialize...")
        time.sleep(2)

        arm_ready = False
        for attempt in range(10):
            try:
                angles = self.mc.get_angles()
                if angles is not None and len(angles) > 0:
                    self.get_logger().info(
                        f"Arm ready after {attempt + 1} attempt(s). "
                        f"Initial angles: {[round(a, 2) for a in angles]}"
                    )
                    arm_ready = True
                    break
            except Exception as e:
                self.get_logger().warning(f"Attempt {attempt + 1}: {e}")
            time.sleep(0.5)

        if not arm_ready:
            self.get_logger().warning(
                "Arm not responding after 10 attempts. Continuing anyway."
            )

        # ROS 2 Subscriber — receive commands from Isaac Sim 
        self.subscription = self.create_subscription(
            JointState,
            '/isaac_joint_commands',
            self.command_callback,
            ARM_QOS
        )

        # ROS 2 Publisher — send arm telemetry to Isaac Sim 
        self.publisher = self.create_publisher(
            JointState,
            '/pi_joint_states',
            ARM_QOS
        )

        # Telemetry Timer — publish at 10Hz 
        self.telemetry_timer = self.create_timer(0.1, self.telemetry_callback)

        self.get_logger().info(
            "Physical arm controller ready. "
            "Listening on /isaac_joint_commands | "
            "Publishing on /pi_joint_states"
        )

    def command_callback(self, msg):
        """Receive joint commands from Isaac Sim and send to physical arm."""
        current_time = time.time()

        if (current_time - self.last_command_time) < self.time_delay:
            return

        if msg.position is None or len(msg.position) == 0:
            self.get_logger().warning("Received empty joint command. Skipping.")
            return

        try:
            joint_angles_deg = [math.degrees(a) for a in msg.position]
            self.get_logger().info(
                f"Commanding arm (deg): {[round(a, 2) for a in joint_angles_deg]}"
            )
            self.mc.send_angles(joint_angles_deg, self.medium_speed)
            self.last_command_time = current_time
        except Exception as e:
            self.get_logger().error(f"Failed to send angles to arm: {e}")

    def telemetry_callback(self):
        """Read arm joint angles and publish as JointState."""
        try:
            angles_deg = self.mc.get_angles()

            if angles_deg is None or len(angles_deg) == 0:
                return

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [
                'joint1', 'joint2', 'joint3',
                'joint4', 'joint5', 'joint6', 'joint7'
            ]
            msg.position = [math.radians(a) for a in angles_deg]
            msg.velocity = []
            msg.effort = []
            self.publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Telemetry read failed: {e}")


def main(args=None):
    rclpy.init(args=args)

    try:
        node = PhysicalArmController()
    except Exception as e:
        print(f"[FATAL] Could not initialize arm controller: {e}")
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested.")
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
