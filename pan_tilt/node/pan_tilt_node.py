#!/usr/bin/env python3
"""
pan_tilt_node.py  –  ROS2 node for a 2-DOF pan/tilt platform
                     using Feetech STS3215 servos on a USB serial bus.

Protocol: SMS/STS half-duplex UART, 1 Mbps default
URT-1 board echoes TX back on RX — echo bytes are consumed explicitly
by counting exact packet length before reading the servo status response.

Published topics:
  ~/joint_states       (sensor_msgs/JointState)  namespaced, reliable
  /joint_states        (sensor_msgs/JointState)  global, BEST_EFFORT for robot_state_publisher
  ~/scan_capture       (sensor_msgs/JointState)  per-step trigger: stamp + commanded pan/tilt
  ~/scan_status        (std_msgs/String)         lifecycle: started / step:i:N / complete / aborted

Subscribed topics:
  ~/pan_tilt_cmd  (geometry_msgs/Vector3)  x=pan_deg, y=tilt_deg, z=speed (0=default)
  ~/joint_cmd     (sensor_msgs/JointState) name+position (rad) generic interface
  ~/scan_start    (geometry_msgs/Vector3)  PAN SWEEP at fixed tilt: x=half_width_deg, y=tilt_deg, z=steps
  ~/nod_start     (geometry_msgs/Vector3)  TILT NOD at pan=0:    x=tilt_min_deg, y=tilt_max_deg, z=steps
  ~/scan_abort    (std_msgs/Empty)         abort an in-progress sweep or nod

Services:
  ~/torque_enable (std_srvs/SetBool)
  ~/go_home       (std_srvs/Trigger)

Parameters:
  serial_port              (string, default /dev/ttyUSB0)
  baud_rate                (int,    default 1000000)
  pan_id                   (int,    default 1)
  tilt_id                  (int,    default 2)
  pan_min_deg              (float,  default -135.0)
  pan_max_deg              (float,  default  135.0)
  tilt_min_deg             (float,  default  -30.0)
  tilt_max_deg             (float,  default   45.0)
  default_speed            (int,    default 500)
  feedback_hz              (float,  default  50.0)
  scan_speed               (int,    default 300)   slower for gentle settling
  scan_settle_sec          (float,  default 0.3)   wait after each step move
  scan_dwell_sec           (float,  default 0.15)  capture window (>= 1 LD19 rotation)
  scan_bigmove_settle_sec  (float,  default 1.0)   wait after initial move to start
  nod_speed                (int,    default 300)   override for nod
  nod_settle_sec           (float,  default 0.3)   override for nod
  nod_dwell_sec            (float,  default 0.15)  override for nod
  nod_bigmove_settle_sec   (float,  default 1.0)   override for nod
"""

import math
import time
import threading
import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3
from std_msgs.msg import String, Empty
from std_srvs.srv import SetBool, Trigger

REG_TORQUE_ENABLE    = 0x28
REG_GOAL_POSITION    = 0x2A
REG_GOAL_SPEED       = 0x2C
REG_PRESENT_POSITION = 0x38

HEADER               = 0xFF
INST_WRITE           = 0x03
INST_READ            = 0x02
INST_SYNC_WRITE      = 0x83
BROADCAST_ID         = 0xFE

TICKS_PER_DEG        = 4096.0 / 360.0
CENTER_TICK          = 2048


class STSBus:
    """
    Feetech STS/SMS half-duplex UART driver.

    The URT-1 board uses a single data line: every byte transmitted by the
    host is echoed back on the RX line before the servo's response arrives.
    We consume the echo by reading exactly len(tx_packet) bytes, then parse
    the servo status packet. No tcflush() is used — it is unreliable on
    CH341 at 1 Mbps. The internal lock serializes all bus access so the
    feedback timer and a running scan thread can share the bus safely.
    """

    def __init__(self, port: str, baud: int = 1_000_000, timeout: float = 0.05):
        self._lock = threading.Lock()
        self._ser  = serial.Serial(
            port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
        )
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()

    def close(self):
        self._ser.close()

    @staticmethod
    def _checksum(servo_id: int, length: int, instruction: int, params: bytes) -> int:
        total = servo_id + length + instruction + sum(params)
        return (~total) & 0xFF

    @staticmethod
    def _build_packet(servo_id: int, instruction: int, params: bytes) -> bytes:
        length = len(params) + 2
        cs     = STSBus._checksum(servo_id, length, instruction, params)
        return bytes([HEADER, HEADER, servo_id, length, instruction]) + params + bytes([cs])

    def _tx(self, pkt: bytes):
        """Send packet and wait for all bytes to be clocked out of the UART."""
        self._ser.write(pkt)
        self._ser.flush()
        time.sleep(len(pkt) * 10e-6 + 0.0002)

    def _read_status(self, echo_len: int = 0) -> bytes | None:
        """
        Consume echo_len echo bytes, then read status packet:
          FF FF ID LEN ERR [PARAMS] CHECKSUM
        Returns param bytes on success, None on timeout / checksum error.
        """
        try:
            if echo_len > 0:
                got = self._ser.read(echo_len)
                if len(got) < echo_len:
                    return None

            hdr = self._ser.read(2)
            if len(hdr) < 2 or hdr[0] != HEADER or hdr[1] != HEADER:
                return None

            meta = self._ser.read(3)
            if len(meta) < 3:
                return None
            sid, length, err = meta

            n_params = length - 2
            if n_params < 0:
                return None

            body = self._ser.read(n_params + 1)
            if len(body) < n_params + 1:
                return None

            params      = body[:n_params]
            received_cs = body[n_params]
            expected_cs = self._checksum(sid, length, 0x00, bytes([err]) + params)
            if received_cs != expected_cs:
                return None
            if err:
                return None
            return params if params else b'\x00'

        except serial.SerialException:
            return None

    def write_byte(self, servo_id: int, reg: int, value: int):
        params = bytes([reg, value & 0xFF])
        pkt    = self._build_packet(servo_id, INST_WRITE, params)
        with self._lock:
            self._tx(pkt)
            self._read_status(echo_len=len(pkt))

    def write_word(self, servo_id: int, reg: int, value: int):
        lo     = value & 0xFF
        hi     = (value >> 8) & 0xFF
        params = bytes([reg, lo, hi])
        pkt    = self._build_packet(servo_id, INST_WRITE, params)
        with self._lock:
            self._tx(pkt)
            self._read_status(echo_len=len(pkt))

    def read_bytes(self, servo_id: int, reg: int, length: int) -> bytes | None:
        params = bytes([reg, length])
        pkt    = self._build_packet(servo_id, INST_READ, params)
        with self._lock:
            self._tx(pkt)
            return self._read_status(echo_len=len(pkt))

    def sync_write_positions(self, servos: list[tuple[int, int, int]]):
        """Command position+speed to multiple servos in one broadcast packet."""
        data_len = 4
        params   = bytes([REG_GOAL_POSITION, data_len])
        for sid, pos, spd in servos:
            pos = max(0, min(4095, pos))
            spd = max(0, min(3000, spd))
            params += bytes([
                sid,
                pos & 0xFF, (pos >> 8) & 0xFF,
                spd & 0xFF, (spd >> 8) & 0xFF,
            ])
        pkt = self._build_packet(BROADCAST_ID, INST_SYNC_WRITE, params)
        with self._lock:
            self._tx(pkt)
            self._ser.read(len(pkt))


def deg_to_tick(deg: float) -> int:
    return int(round(CENTER_TICK + deg * TICKS_PER_DEG))

def tick_to_deg(tick: int) -> float:
    return (tick - CENTER_TICK) / TICKS_PER_DEG

def deg_to_rad(deg: float) -> float:
    return deg * math.pi / 180.0

def rad_to_deg(rad: float) -> float:
    return rad * 180.0 / math.pi


class PanTiltNode(Node):

    def __init__(self):
        super().__init__('pan_tilt_node')

        self.declare_parameter('serial_port',   '/dev/ttyUSB0')
        self.declare_parameter('baud_rate',     1_000_000)
        self.declare_parameter('pan_id',        1)
        self.declare_parameter('tilt_id',       2)
        self.declare_parameter('pan_min_deg',  -135.0)
        self.declare_parameter('pan_max_deg',   135.0)
        self.declare_parameter('tilt_min_deg',  -30.0)
        self.declare_parameter('tilt_max_deg',   45.0)
        self.declare_parameter('default_speed',  500)
        self.declare_parameter('feedback_hz',    50.0)
        self.declare_parameter('scan_speed',     300)
        self.declare_parameter('scan_settle_sec',         0.3)
        self.declare_parameter('scan_dwell_sec',          0.15)
        self.declare_parameter('scan_bigmove_settle_sec', 1.0)
        self.declare_parameter('nod_speed',     300)
        self.declare_parameter('nod_settle_sec',          0.3)
        self.declare_parameter('nod_dwell_sec',           0.15)
        self.declare_parameter('nod_bigmove_settle_sec',  1.0)

        port           = self.get_parameter('serial_port').value
        baud           = self.get_parameter('baud_rate').value
        self._pan_id   = self.get_parameter('pan_id').value
        self._tilt_id  = self.get_parameter('tilt_id').value
        self._pan_min  = self.get_parameter('pan_min_deg').value
        self._pan_max  = self.get_parameter('pan_max_deg').value
        self._tilt_min = self.get_parameter('tilt_min_deg').value
        self._tilt_max = self.get_parameter('tilt_max_deg').value
        self._def_spd  = self.get_parameter('default_speed').value
        fb_hz          = self.get_parameter('feedback_hz').value
        self._scan_spd     = self.get_parameter('scan_speed').value
        self._scan_settle  = self.get_parameter('scan_settle_sec').value
        self._scan_dwell   = self.get_parameter('scan_dwell_sec').value
        self._scan_bigmove = self.get_parameter('scan_bigmove_settle_sec').value
        self._nod_spd      = self.get_parameter('nod_speed').value
        self._nod_settle   = self.get_parameter('nod_settle_sec').value
        self._nod_dwell    = self.get_parameter('nod_dwell_sec').value
        self._nod_bigmove  = self.get_parameter('nod_bigmove_settle_sec').value

        self.get_logger().info(f'Opening STS bus on {port} @ {baud} baud')
        try:
            self._bus = STSBus(port, baud)
        except serial.SerialException as e:
            self.get_logger().fatal(f'Cannot open serial port: {e}')
            raise SystemExit(1)

        self._set_torque(True)
        self.get_logger().info(
            f'Pan/tilt ready  pan_id={self._pan_id}  tilt_id={self._tilt_id}')

        qos_reliable    = QoSProfile(depth=10)
        qos_best_effort = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._js_pub        = self.create_publisher(JointState, '~/joint_states', qos_reliable)
        self._js_pub_global = self.create_publisher(JointState, '/joint_states',  qos_best_effort)

        self._capture_pub = self.create_publisher(JointState, '~/scan_capture', qos_reliable)
        self._status_pub  = self.create_publisher(String,     '~/scan_status',  qos_reliable)

        self.create_subscription(Vector3,    '~/pan_tilt_cmd', self._cmd_cb,        10)
        self.create_subscription(JointState, '~/joint_cmd',    self._joint_cmd_cb,  10)
        self.create_subscription(Vector3,    '~/scan_start',   self._scan_start_cb, 10)
        self.create_subscription(Vector3,    '~/nod_start',    self._nod_start_cb,  10)
        self.create_subscription(Empty,      '~/scan_abort',   self._scan_abort_cb, 10)

        self.create_service(SetBool, '~/torque_enable', self._torque_srv)
        self.create_service(Trigger, '~/go_home',       self._home_srv)

        self.create_timer(1.0 / fb_hz, self._feedback_cb)

        self._pan_deg  = 0.0
        self._tilt_deg = 0.0

        self._scan_thread = None
        self._abort_evt   = threading.Event()

    def destroy_node(self):
        try:
            self._abort_evt.set()
            if self._scan_thread and self._scan_thread.is_alive():
                self._scan_thread.join(timeout=2.0)
            self._set_torque(False)
            self._bus.close()
        except Exception:
            pass
        super().destroy_node()

    def _clamp_pan(self, deg):
        return max(self._pan_min, min(self._pan_max, deg))

    def _clamp_tilt(self, deg):
        return max(self._tilt_min, min(self._tilt_max, deg))

    def _move(self, pan_deg: float, tilt_deg: float, speed: int = 0):
        pan_deg  = self._clamp_pan(pan_deg)
        tilt_deg = self._clamp_tilt(tilt_deg)
        spd = speed if speed > 0 else self._def_spd
        self._bus.sync_write_positions([
            (self._pan_id,  deg_to_tick(-pan_deg),  spd),
            (self._tilt_id, deg_to_tick(tilt_deg),  spd),
        ])
        self._pan_deg  = pan_deg
        self._tilt_deg = tilt_deg

    def _set_torque(self, enable: bool):
        val = 1 if enable else 0
        self._bus.write_byte(self._pan_id,  REG_TORQUE_ENABLE, val)
        self._bus.write_byte(self._tilt_id, REG_TORQUE_ENABLE, val)

    def _read_position(self, servo_id: int) -> float | None:
        data = self._bus.read_bytes(servo_id, REG_PRESENT_POSITION, 2)
        if data and len(data) >= 2:
            tick = data[0] | (data[1] << 8)
            return tick_to_deg(tick)
        return None

    @staticmethod
    def _linspace(start: float, stop: float, n: int) -> list[float]:
        if n <= 1:
            return [start]
        step = (stop - start) / (n - 1)
        return [start + step * i for i in range(n)]

    def _status(self, text: str):
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)
        self.get_logger().info(f'scan: {text}')

    def _cmd_cb(self, msg: Vector3):
        self._move(msg.x, msg.y, int(msg.z))

    def _joint_cmd_cb(self, msg: JointState):
        pan_deg  = self._pan_deg
        tilt_deg = self._tilt_deg
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                deg = rad_to_deg(msg.position[i])
                if name == 'pan':
                    pan_deg = deg
                elif name == 'tilt':
                    tilt_deg = deg
        self._move(pan_deg, tilt_deg)

    def _busy(self) -> bool:
        return self._scan_thread is not None and self._scan_thread.is_alive()

    def _scan_start_cb(self, msg: Vector3):
        """
        PAN SWEEP at fixed tilt.
        Vector3: x = half-width deg (sweep -x..+x), y = tilt deg, z = steps
        """
        if self._busy():
            self.get_logger().warn('motion already running; ignoring scan_start')
            return
        half_width = float(msg.x)
        tilt_deg   = float(msg.y)
        steps      = max(2, int(msg.z))
        self._abort_evt.clear()
        self._scan_thread = threading.Thread(
            target=self._run_pan_sweep,
            args=(half_width, tilt_deg, steps),
            daemon=True,
        )
        self._scan_thread.start()

    def _nod_start_cb(self, msg: Vector3):
        """
        TILT NOD at pan=0 (front-facing quick vertical scan).
        Vector3: x = tilt_min deg, y = tilt_max deg, z = steps
        """
        if self._busy():
            self.get_logger().warn('motion already running; ignoring nod_start')
            return
        tilt_lo = float(msg.x)
        tilt_hi = float(msg.y)
        steps   = max(2, int(msg.z))
        self._abort_evt.clear()
        self._scan_thread = threading.Thread(
            target=self._run_tilt_nod,
            args=(tilt_lo, tilt_hi, steps),
            daemon=True,
        )
        self._scan_thread.start()

    def _scan_abort_cb(self, _msg: Empty):
        if self._busy():
            self.get_logger().warn('abort requested')
            self._abort_evt.set()

    def _publish_capture(self, index: int, total: int, pan_deg: float, tilt_deg: float):
        """Per-step trigger: stamp + commanded angles. Assembler matches /scan by stamp."""
        cap = JointState()
        cap.header.stamp = self.get_clock().now().to_msg()
        cap.header.frame_id = 'base_link'
        cap.name     = ['pan', 'tilt']
        cap.position = [deg_to_rad(pan_deg), deg_to_rad(tilt_deg)]
        cap.velocity = [float(index), float(total)]
        self._capture_pub.publish(cap)

    def _abort_home(self) -> bool:
        self._status('aborted')
        self._move(0.0, 0.0)
        return True

    def _stepped_capture(self, positions, fixed_pan, fixed_tilt, is_pan_axis,
                         speed, settle, dwell):
        """
        Walk through `positions`. For a pan sweep, positions are pan angles and
        fixed_tilt holds; for a tilt nod, positions are tilt angles and fixed_pan
        holds. Captures at each settled step. Returns False if aborted.
        """
        total = len(positions)
        for i, p in enumerate(positions):
            if self._abort_evt.is_set():
                self._abort_home()
                return False

            if is_pan_axis:
                pan, tilt = p, fixed_tilt
            else:
                pan, tilt = fixed_pan, p
            self._move(pan, tilt, speed=speed)

            if self._abort_evt.wait(settle):
                self._abort_home()
                return False

            self._publish_capture(i, total, pan, tilt)
            self._status(f'step:{i}:{total}')

            if self._abort_evt.wait(dwell):
                self._abort_home()
                return False
        return True

    def _run_pan_sweep(self, half_width: float, tilt_deg: float, steps: int):
        """Pan sweep -half_width..+half_width at fixed tilt. Runs in a thread."""
        try:
            self._status('started')
            self._move(-half_width, tilt_deg, speed=self._scan_spd)
            if self._abort_evt.wait(self._scan_bigmove):
                self._abort_home(); return
            positions = self._linspace(-half_width, half_width, steps)
            if not self._stepped_capture(positions, 0.0, tilt_deg, True,
                                         self._scan_spd, self._scan_settle, self._scan_dwell):
                return
            self._move(0.0, 0.0)
            self._status('complete')
        except Exception as e:
            self.get_logger().error(f'pan sweep error: {e}')
            self._status('aborted')

    def _run_tilt_nod(self, tilt_lo: float, tilt_hi: float, steps: int):
        """Tilt nod tilt_lo..tilt_hi at pan=0 (front-facing). Runs in a thread."""
        try:
            self._status('started')
            self._move(0.0, tilt_lo, speed=self._nod_spd)
            if self._abort_evt.wait(self._nod_bigmove):
                self._abort_home(); return
            positions = self._linspace(tilt_lo, tilt_hi, steps)
            if not self._stepped_capture(positions, 0.0, 0.0, False,
                                         self._nod_spd, self._nod_settle, self._nod_dwell):
                return
            self._move(0.0, 0.0)
            self._status('complete')
        except Exception as e:
            self.get_logger().error(f'tilt nod error: {e}')
            self._status('aborted')

    def _feedback_cb(self):
        pan_raw  = self._read_position(self._pan_id)
        pan_deg  = -pan_raw if pan_raw is not None else None
        tilt_deg = self._read_position(self._tilt_id)

        if pan_deg  is None: pan_deg  = self._pan_deg
        if tilt_deg is None: tilt_deg = self._tilt_deg

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.header.frame_id = 'base_link'
        js.name     = ['pan', 'tilt']
        js.position = [deg_to_rad(pan_deg), deg_to_rad(tilt_deg)]

        self._js_pub.publish(js)
        self._js_pub_global.publish(js)

    def _torque_srv(self, request, response):
        self._set_torque(request.data)
        response.success = True
        response.message = 'torque enabled' if request.data else 'torque disabled'
        return response

    def _home_srv(self, request, response):
        self._move(0.0, 0.0)
        response.success = True
        response.message = 'homing to 0,0'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PanTiltNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
