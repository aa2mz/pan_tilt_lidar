#!/bin/bash
# scanner.sh  –  bring up the full scanning subsystem
# usage: scanner.sh <lidar_usb_n> <servo_usb_n>
# ex:    scanner.sh 0 1   (rtk not running: lidar=USB0, servo=USB1)
#        scanner.sh 1 2   (rtk on USB0:     lidar=USB1, servo=USB2)

if [ "$#" -ne 2 ]; then
    echo "usage: scanner.sh <lidar_usb_n> <servo_usb_n>"
    echo "  ex:  scanner.sh 0 1   (no rtk)"
    echo "       scanner.sh 1 2   (rtk on USB0)"
    exit 1
fi

LIDAR_PORT=/dev/ttyUSB${1}
SERVO_PORT=/dev/ttyUSB${2}
ROVER_IP=192.168.4.110
MAP_DIR=/home/ed/2081_map
NODE_DIR=$MAP_DIR/pan_tilt/node
RVIZ_CONFIG=$MAP_DIR/odom_scan.rviz
URDF=$MAP_DIR/pan_tilt_lidar.urdf

# ── log management ────────────────────────────────────────────────────────────
# Send ROS 2 logs to tmpfs so they never fill the SD card and clear on reboot.
export ROS_LOG_DIR=/tmp/roslog
mkdir -p "$ROS_LOG_DIR"
find "$ROS_LOG_DIR" -maxdepth 1 -type d -mmin +60 -exec rm -rf {} + 2>/dev/null

echo "=== scanner startup ==="
echo "  lidar : $LIDAR_PORT"
echo "  servo : $SERVO_PORT"
echo "  rover : $ROVER_IP"
echo "  urdf  : $URDF"
echo "  logs  : $ROS_LOG_DIR (tmpfs, auto-pruned)"
echo "======================="

if [ ! -f "$URDF" ]; then
    echo "ERROR: URDF not found at $URDF"
    exit 1
fi

source /opt/ros/jazzy/setup.bash
source /home/ed/ros2_ws/install/setup.bash

PIDS=()
TMPFILES=()

cleanup() {
    echo ""
    echo "shutting down..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null
    done
    wait 2>/dev/null
    for f in "${TMPFILES[@]}"; do
        rm -f "$f" 2>/dev/null
    done
    echo "done."
}
trap cleanup EXIT INT TERM

# ── odom bridge ───────────────────────────────────────────────────────────────
echo "[1/7] odom_bridge"
ROVERMQTT=$ROVER_IP python3 $MAP_DIR/odom_bridge.py &
PIDS+=($!)
sleep 1

# ── robot_state_publisher ─────────────────────────────────────────────────────
# Load the URDF via a YAML params file, NOT a -p command-line override.
# Passing multi-line XML (especially with comments) through -p breaks the rcl
# argument parser. A params file carries the XML as a literal block scalar.
echo "[2/7] robot_state_publisher"
RSP_PARAMS=$(mktemp /tmp/rsp_params.XXXXXX.yaml)
TMPFILES+=("$RSP_PARAMS")
{
    echo "robot_state_publisher:"
    echo "  ros__parameters:"
    echo "    robot_description: |"
    sed 's/^/      /' "$URDF"      # indent each URDF line under the block scalar
} > "$RSP_PARAMS"
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args --params-file "$RSP_PARAMS" &
PIDS+=($!)
sleep 1

# ── ldlidar ───────────────────────────────────────────────────────────────────
# laser_scan_dir:=false  -> correct (un-mirrored) left/right sweep
echo "[3/7] ldlidar on $LIDAR_PORT"
ros2 run ldlidar_stl_ros2 ldlidar_stl_ros2_node \
    --ros-args \
    -r __name:=LD19 \
    -p product_name:=LDLiDAR_LD19 \
    -p topic_name:=scan \
    -p frame_id:=base_laser \
    -p port_name:=$LIDAR_PORT \
    -p port_baudrate:=230400 \
    -p laser_scan_dir:=true \
    -p enable_angle_crop_func:=false &
PIDS+=($!)
sleep 2

# ── pan/tilt node ─────────────────────────────────────────────────────────────
echo "[4/7] pan_tilt_node on $SERVO_PORT"
python3 $NODE_DIR/pan_tilt_node.py \
    --ros-args \
    -p serial_port:=$SERVO_PORT \
    -p pan_id:=12 \
    -p tilt_id:=11 \
    -p feedback_hz:=50.0 &
PIDS+=($!)
sleep 1

# ── scan assembler ────────────────────────────────────────────────────────────
echo "[5/7] scan_assembler"
python3 $MAP_DIR/scan_assembler.py &
PIDS+=($!)
sleep 1

# ── scan MQTT bridge ──────────────────────────────────────────────────────────
echo "[6/7] scan_mqtt_bridge"
ROVERMQTT=$ROVER_IP python3 $MAP_DIR/scan_mqtt_bridge.py &
PIDS+=($!)
sleep 1

# ── rviz2 (skip when headless: set HEADLESS=1 or no DISPLAY) ───────────────────
if [ "${HEADLESS:-0}" = "1" ] || [ -z "${DISPLAY:-}" ]; then
    echo "[7/7] rviz2 — SKIPPED (headless)"
else
    echo "[7/7] rviz2"
    rviz2 -d $RVIZ_CONFIG &
    PIDS+=($!)
fi

echo ""
echo "all up. Ctrl-C to stop everything."
wait
