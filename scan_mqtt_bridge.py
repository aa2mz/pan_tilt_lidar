#!/usr/bin/env python3
"""
scan_mqtt_bridge.py — MQTT <-> ROS2 bridge for the pan/tilt scan subsystem.

Translates rover scan commands from the RP2040 (over MQTT) into the ROS2
scan-trigger topics the pan_tilt_node already exposes, and reports completion
back to the RP2040 over MQTT.

MQTT in  (from RP2040):
  rover/scan/pan   "<half_width> <tilt> <steps>  [comment...]"   -> ROS2 /pan_tilt_node/scan_start  (pan sweep)
  rover/scan/tilt  "<tilt_lo>    <tilt_hi> <steps> [comment...]" -> ROS2 /pan_tilt_node/nod_start   (tilt nod)

  Payloads are space-separated ascii floats in DEGREES. Anything after the
  first three numbers is treated as a comment and ignored.

MQTT out (to RP2040):
  rover/scan/complete  "ok"        when a scan finishes normally
  rover/scan/complete  "aborted"   when a scan is aborted

ROS2 in:
  /pan_tilt_node/scan_status  (std_msgs/String)  'started'/'step:i:N'/'complete'/'aborted'

ROS2 out:
  /pan_tilt_node/scan_start   (geometry_msgs/Vector3)  x=half_width y=tilt z=steps
  /pan_tilt_node/nod_start    (geometry_msgs/Vector3)  x=tilt_lo y=tilt_hi z=steps

Environment:
  ROVERMQTT : broker hostname or IP (default port 1883)

Run:
  source /opt/ros/jazzy/setup.bash
  ROVERMQTT=<broker> python3 scan_mqtt_bridge.py
"""

import os
import sys
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import String

import paho.mqtt.client as mqtt


BROKER = os.environ.get("ROVERMQTT")
PORT   = 1883

# MQTT topics
MQTT_PAN      = "rover/scan/pan"
MQTT_TILT     = "rover/scan/tilt"
MQTT_COMPLETE = "rover/scan/complete"

# ROS2 topics
ROS_SCAN_START = "/pan_tilt_node/scan_start"
ROS_NOD_START  = "/pan_tilt_node/nod_start"
ROS_STATUS     = "/pan_tilt_node/scan_status"


def die(msg: str) -> None:
    print(f"scan_mqtt_bridge: {msg}", file=sys.stderr)
    sys.exit(1)


if not BROKER:
    die("ROVERMQTT is not set")


def parse_xyz(payload: bytes):
    """
    Parse "<x> <y> <z> [comment...]" ascii floats (degrees).
    Returns (x, y, z) or None if the first three tokens aren't all floats.
    Anything after the first three tokens is ignored.
    """
    try:
        text  = payload.decode("ascii").rstrip("\x00")
        parts = text.split()
        if len(parts) < 3:
            return None
        x = float(parts[0])
        y = float(parts[1])
        z = float(parts[2])
        return (x, y, z)
    except Exception:
        return None


class ScanMqttBridge(Node):

    def __init__(self):
        super().__init__("scan_mqtt_bridge")

        self._scan_pub = self.create_publisher(Vector3, ROS_SCAN_START, 10)
        self._nod_pub  = self.create_publisher(Vector3, ROS_NOD_START,  10)

        self.create_subscription(String, ROS_STATUS, self._status_cb, 10)

        # set by make_mqtt_client once the client exists
        self._mqtt = None

        self.get_logger().info(
            f"bridge ready: MQTT {MQTT_PAN}/{MQTT_TILT} -> ROS2 scan triggers")

    def attach_mqtt(self, client: mqtt.Client):
        self._mqtt = client

    # ── MQTT -> ROS2 ──────────────────────────────────────────────────────────

    def trigger_pan(self, x: float, y: float, z: float):
        msg = Vector3(x=x, y=y, z=z)
        self._scan_pub.publish(msg)
        self.get_logger().info(f"pan sweep: half_width={x} tilt={y} steps={z}")

    def trigger_nod(self, x: float, y: float, z: float):
        msg = Vector3(x=x, y=y, z=z)
        self._nod_pub.publish(msg)
        self.get_logger().info(f"tilt nod: lo={x} hi={y} steps={z}")

    # ── ROS2 -> MQTT ──────────────────────────────────────────────────────────

    def _status_cb(self, msg: String):
        text = msg.data
        if text == "complete":
            self._report("ok")
        elif text == "aborted":
            self._report("aborted")
        # 'started' and 'step:i:N' are not reported upstream

    def _report(self, result: str):
        if self._mqtt is None:
            return
        try:
            self._mqtt.publish(MQTT_COMPLETE, result, qos=0)
            self.get_logger().info(f"reported scan {result} -> {MQTT_COMPLETE}")
        except Exception as exc:
            self.get_logger().error(f"failed to publish complete: {exc}")


def make_mqtt_client(node: ScanMqttBridge) -> mqtt.Client:

    def on_connect(client, userdata, flags, rc):
        if rc != 0:
            node.get_logger().error(f"MQTT connect failed rc={rc}")
            return
        client.subscribe(MQTT_PAN,  qos=0)
        client.subscribe(MQTT_TILT, qos=0)
        node.get_logger().info(
            f"MQTT connected, subscribed to {MQTT_PAN} and {MQTT_TILT}")

    def on_message(client, userdata, msg):
        xyz = parse_xyz(msg.payload)
        if xyz is None:
            node.get_logger().warning(
                f"bad scan payload on {msg.topic}: {msg.payload!r}")
            return
        x, y, z = xyz
        if msg.topic == MQTT_PAN:
            node.trigger_pan(x, y, z)
        elif msg.topic == MQTT_TILT:
            node.trigger_nod(x, y, z)
        else:
            node.get_logger().warning(f"unexpected topic: {msg.topic}")

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
    node = ScanMqttBridge()

    client = make_mqtt_client(node)
    node.attach_mqtt(client)

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
