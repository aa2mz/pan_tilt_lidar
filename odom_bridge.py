#!/usr/bin/env python3
"""
odom_bridge.py — MQTT rover/odom → ROS2 nav_msgs/Odometry

Subscribes to MQTT topic rover/odom
Payload: "<x> <y> <theta>"  (meters, meters, radians — 0=East, standard math)
Publishes: nav_msgs/Odometry on ROS2 topic /odom, frame odom

Environment:
  ROVERMQTT : broker hostname or IP (default port 1883)

Run:
  source /opt/ros/jazzy/setup.bash
  pip install paho-mqtt --break-system-packages   # if not already installed
  ROVERMQTT=<broker> python3 odom_bridge.py
"""

import os
import sys
import math
import threading

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

import paho.mqtt.client as mqtt


BROKER    = os.environ.get("ROVERMQTT")
PORT      = 1883
ODOM_SUB  = "rover/odom"
ROS_TOPIC = "/odom"
FRAME_ID  = "odom"


def die(msg: str) -> None:
    print(f"odom_bridge: {msg}", file=sys.stderr)
    sys.exit(1)


if not BROKER:
    die("ROVERMQTT is not set")


def heading_to_quaternion(theta_rad: float) -> Quaternion:
    """
    Convert a 2D heading (radians, 0=East/+X, CCW positive)
    to a ROS2 quaternion (rotation about Z axis).
    """
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(theta_rad / 2.0)
    q.w = math.cos(theta_rad / 2.0)
    return q


class OdomBridge(Node):

    def __init__(self):
        super().__init__("odom_bridge")
        self._pub = self.create_publisher(Odometry, ROS_TOPIC, 10)
        self.get_logger().info(f"publishing odometry on {ROS_TOPIC}")

    def publish_odom(self, x: float, y: float, theta: float) -> None:
        msg = Odometry()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = FRAME_ID
        msg.child_frame_id  = "base_link"

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = heading_to_quaternion(theta)

        self._pub.publish(msg)


def make_mqtt_client(node: OdomBridge) -> mqtt.Client:

    def on_connect(client, userdata, flags, rc):
        if rc != 0:
            node.get_logger().error(f"MQTT connect failed rc={rc}")
            return
        client.subscribe(ODOM_SUB, qos=0)
        node.get_logger().info(f"MQTT connected, subscribed to {ODOM_SUB}")

    def on_message(client, userdata, msg):
        try:
            parts = msg.payload.decode("ascii").rstrip("\x00").split()
            if len(parts) < 3:
                node.get_logger().warning(f"unexpected odom payload: {msg.payload!r}")
                return
            x, y, theta = float(parts[0]), float(parts[1]), float(parts[2])
        except Exception as exc:
            node.get_logger().error(f"odom parse error: {exc}")
            return

        node.publish_odom(x, y, theta)

    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.reconnect_delay_set(min_delay=1, max_delay=30)

    try:
        client.connect(BROKER, PORT, keepalive=60)
    except Exception as exc:
        die(f"cannot connect to MQTT broker {BROKER}:{PORT}: {exc}")

    return client


def main() -> None:
    rclpy.init()
    node = OdomBridge()

    client = make_mqtt_client(node)

    # MQTT loop on its own thread; ROS2 spins on main thread.
    mqtt_thread = threading.Thread(
        target=client.loop_forever,
        kwargs={"retry_first_connection": True},
        daemon=True,
        name="mqtt-loop",
    )
    mqtt_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        client.disconnect()


if __name__ == "__main__":
    main()
