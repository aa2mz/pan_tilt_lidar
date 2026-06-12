# ros2_pan_tilt_lidar

A ROS 2 (Jazzy) subsystem that turns a 2-DOF pan/tilt platform and a 2D LiDAR
into a 3D scanner. Two Feetech STS3215 serial-bus servos aim a single
LDROBOT LD19 LiDAR; the node sweeps the platform, and an assembler stacks the
2D scans into a 3D point cloud using the live TF tree. An optional MQTT bridge
lets an external controller (e.g. a microcontroller mission queue) trigger
scans and receive completion reports.

No vendor SDK is required — the Feetech STS/SMS half-duplex serial protocol is
implemented directly over `pyserial`.

## Features

- Direct STS3215 driver (no external SDK); handles the URT-1 half-duplex TX echo
- `SYNC_WRITE` coordinated motion for both servos in a single packet
- Live joint feedback published as `JointState` for `robot_state_publisher`
- Two scan modes: a **pan sweep** at fixed tilt, and a quick front-facing **tilt nod**
- Stop-and-capture sequencing with per-step settle and dwell, all tunable at runtime
- 3D point-cloud assembler that transforms each 2D scan through TF at capture time
- Abortable sweeps; clean lifecycle status reporting
- Optional MQTT bridge for remote trigger and completion handshake

## Hardware

- 2x Feetech STS3215 servos on a shared serial bus (pan + tilt)
- Feetech URT-1 USB serial adapter (CH341, half-duplex)
- LDROBOT LD19 2D LiDAR
- Any Linux SBC running ROS 2 Jazzy (developed on an RK3588)

Servo zero positions are set in hardware using the Feetech tool (write the
`Offset` register so the desired forward orientation reads tick 2048). The node
assumes tick 2048 = 0 degrees for each axis.

## Components

| File | Role |
|------|------|
| `pan_tilt_node.py`     | STS3215 driver + ROS 2 node: motion, feedback, scan sequencer |
| `scan_assembler.py`    | Accumulates captured 2D scans into a 3D `PointCloud2` via TF |
| `scan_mqtt_bridge.py`  | MQTT <-> ROS 2 bridge for remote scan trigger and completion |
| `pan_tilt_lidar.urdf`  | Kinematic chain `base_link -> pan_link -> tilt_link -> base_laser` |
| `scanner.sh`           | Brings up the whole stack in one command |

## Dependencies

- ROS 2 Jazzy (`rclpy`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, `std_srvs`, `tf2_ros`, `robot_state_publisher`)
- `pyserial`
- `paho-mqtt` (only for the MQTT bridge)
- A 2D LiDAR driver publishing `sensor_msgs/LaserScan` on `/scan`. The assembler is driver-agnostic — any conformant 2D LiDAR works. The bundled `scanner.sh` targets the LDROBOT LD19 via a patched
  `ldlidar_stl_ros2` (see [LiDAR driver](#lidar-driver) below).

```bash
pip install pyserial paho-mqtt --break-system-packages
```

### LiDAR driver

`/scan` (`sensor_msgs/LaserScan`) is the standard ROS 2 interface for 2D LiDAR,
so the assembler will consume scans from any conformant driver — only the
launch parameters and the mounting frame are LiDAR-specific. The bundled
`scanner.sh` initializes the LDROBOT **LD19** using the `ldlidar_stl_ros2`
driver.

Upstream `ldlidar_stl_ros2` fails to build on recent GCC/glibc with a missing
`pthread.h` include (`pthread_mutex_init was not declared in this scope`). A
patched fork with the one-line fix committed is here:

```bash
cd ~/ros2_ws/src
git clone https://github.com/aa2mz/ldlidar_stl_ros2.git
cd ~/ros2_ws
colcon build --packages-select ldlidar_stl_ros2
source install/setup.bash
```

To use a different LiDAR instead, replace the ldlidar block in `scanner.sh`
with your driver's launch/run command, and make sure it publishes
`LaserScan` on `/scan` with a `frame_id` that exists in the TF tree (the URDF
expects `base_laser` — either emit that frame or adjust the URDF's
`lidar_joint` child link to match).

## Quick start

Run the pan/tilt node directly (the package does not need to be built/installed):

```bash
python3 pan_tilt_node.py --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p pan_id:=12 -p tilt_id:=11 \
  -p feedback_hz:=50.0
```

Point the platform (degrees; x=pan, y=tilt, z=speed, 0=default):

```bash
ros2 topic pub --once /pan_tilt_node/pan_tilt_cmd \
  geometry_msgs/msg/Vector3 "{x: 30.0, y: -15.0, z: 300.0}"
```

Bring up the full 3D-scan stack:

```bash
./scanner.sh <lidar_usb_n> <servo_usb_n>
# e.g.  ./scanner.sh 0 1
```

`scanner.sh` takes the two USB port numbers as arguments because device order
depends on what else is plugged in (an RTK GPS, for example, may claim
`ttyUSB0`). Pass the LiDAR port first, the servo bus second.

## Scan commands

A **pan sweep** sweeps pan from `-half_width` to `+half_width` at a fixed tilt:

```bash
# half_width=45 deg, tilt=-30 deg, 60 steps
ros2 topic pub --once /pan_tilt_node/scan_start \
  geometry_msgs/msg/Vector3 "{x: 45.0, y: -30.0, z: 60.0}"
```

A **tilt nod** sweeps tilt at pan=0, for a quick front-facing vertical scan:

```bash
# tilt_lo=-10 deg, tilt_hi=40 deg, 50 steps (~1 deg/step)
ros2 topic pub --once /pan_tilt_node/nod_start \
  geometry_msgs/msg/Vector3 "{x: -10.0, y: 40.0, z: 50.0}"
```

Abort an in-progress scan:

```bash
ros2 topic pub --once /pan_tilt_node/scan_abort std_msgs/msg/Empty "{}"
```

The assembler publishes the finished cloud (latched) on
`/scan_assembler/cloud`. In RViz2, add a `PointCloud2` display on that topic
with fixed frame `base_link`.

## MQTT interface

Set the broker via the `ROVERMQTT` environment variable (port 1883, no auth):

```bash
ROVERMQTT=192.168.4.110 python3 scan_mqtt_bridge.py
```

Payloads are space-separated ascii floats in **degrees**. Anything after the
first three numbers is treated as a comment and ignored.

| Direction | Topic | Payload | Effect |
|-----------|-------|---------|--------|
| in  | `rover/scan/pan`      | `<half_width> <tilt> <steps>` | pan sweep |
| in  | `rover/scan/tilt`     | `<tilt_lo> <tilt_hi> <steps>` | tilt nod |
| out | `rover/scan/complete` | `ok` / `aborted`              | sent when a scan finishes |

```bash
mosquitto_pub -h 192.168.4.110 -t rover/scan/pan -m "45 -30 60 facade sweep"
mosquitto_sub -h 192.168.4.110 -t rover/scan/complete
```

## ROS 2 interface

### pan_tilt_node

Published:

- `~/joint_states` (`sensor_msgs/JointState`) — namespaced, reliable
- `/joint_states` (`sensor_msgs/JointState`) — global, BEST_EFFORT, for `robot_state_publisher`
- `~/scan_capture` (`sensor_msgs/JointState`) — per-step capture trigger (stamp + commanded angles)
- `~/scan_status` (`std_msgs/String`) — `started` / `step:i:N` / `complete` / `aborted`

Subscribed:

- `~/pan_tilt_cmd` (`geometry_msgs/Vector3`) — x=pan, y=tilt, z=speed (0=default)
- `~/joint_cmd` (`sensor_msgs/JointState`) — name+position (rad) interface
- `~/scan_start` (`geometry_msgs/Vector3`) — pan sweep: x=half_width, y=tilt, z=steps
- `~/nod_start` (`geometry_msgs/Vector3`) — tilt nod: x=tilt_lo, y=tilt_hi, z=steps
- `~/scan_abort` (`std_msgs/Empty`)

Services:

- `~/torque_enable` (`std_srvs/SetBool`)
- `~/go_home` (`std_srvs/Trigger`)

### Key parameters

| Parameter | Default | Notes |
|-----------|---------|-------|
| `serial_port` | `/dev/ttyUSB0` | servo bus |
| `pan_id` / `tilt_id` | `1` / `2` | servo IDs on the bus |
| `pan_min_deg` / `pan_max_deg` | `-135` / `135` | software stops |
| `tilt_min_deg` / `tilt_max_deg` | `-30` / `45` | software stops |
| `feedback_hz` | `50.0` | keep high so TF leads the scan stamps |
| `scan_speed` | `300` | raw STS speed units for scan moves |
| `scan_settle_sec` | `0.3` | wait after each step before capture |
| `scan_dwell_sec` | `0.15` | capture window (>= one LiDAR rotation) |
| `scan_bigmove_settle_sec` | `1.0` | wait after the initial move to scan start |
| `nod_*` | match scan | independent overrides for the nod mode |

## How it fits together

```
                 MQTT (rover/scan/*)
                        |
                 scan_mqtt_bridge
                        |
   /pan_tilt_node/scan_start | nod_start | scan_abort
                        |
                  pan_tilt_node  ──► STS3215 servos (serial)
                   |        |
       /joint_states     ~/scan_status, ~/scan_capture
            |                       |
   robot_state_publisher      scan_assembler ◄── /scan (LD19)
            |                       |
           TF ─────────────────────┘
                        |
              /scan_assembler/cloud  ──►  RViz2
```

Each 2D scan is transformed into `base_link` using the TF available when it is
captured. Because the platform is settled and stationary at each step, the
assembler looks up the latest transform rather than the scan's exact stamp,
which avoids a timestamp race while remaining geometrically correct.

## Design notes

- **Why no SDK:** the STS/SMS protocol is simple enough to implement directly,
  and doing so removes a dependency and makes the half-duplex echo handling
  explicit and debuggable.
- **Half-duplex echo:** the URT-1 echoes every transmitted byte back on the RX
  line. The driver consumes exactly the transmitted byte count before reading
  the servo's status packet, rather than relying on `tcflush` (unreliable on
  CH341 at 1 Mbps).
- **Stop-and-capture vs continuous:** scanning while stationary at each step
  keeps the geometry simple and the cloud clean. The dwell only needs to be
  long enough to catch one full LiDAR rotation.
- **2D for driving, 3D when stopped:** the LD19 stays fixed and forward for
  obstacle detection while moving; the 3D sweep is a separate, deliberate
  operation performed when the platform is stopped.

## Roadmap / TODO

- [ ] `rover/scan/point` MQTT topic: aim the platform and hold (maps to `pan_tilt_cmd`)
- [ ] Scan-collision mitigation: have direct point/aim commands respect the
      sequencer busy flag so a manual aim can't fight a running sweep
- [ ] `rover/scan/abort` over MQTT
- [ ] Reject (rather than silently clamp) out-of-range commanded angles
- [ ] Read servo temperature / error flags and surface faults
- [ ] Optional IMU-gated settle detection in place of the fixed dwell
- [ ] Multi-pose cloud accumulation / hand-off to a mapping node

## License

Copyright 2024-2026 Edward L. Taychert, AA2MZ.

This project is licensed to you under the terms of the **GNU General Public
License**, either version 3 of the License, or (at your option) any later
version. Some portions are licensed under the **GNU Lesser General Public
License**, version 3 or later — see the license notice in each individual file.

My intent is that you may use and redistribute this software freely, as long as
you adhere to the terms of the GNU licenses. See https://www.gnu.org/licenses/
for the full text. If you need different licensing terms, please get in touch.

The full GPL-3.0 text is in [`LICENSE`](LICENSE).

*And if this saved you some trouble — consider buying me a coffee, or sending
electronic parts for me to play with.*
