import math
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster

try:
    import serial
except ImportError as exc:  # pragma: no cover
    raise SystemExit("pyserial is required: add python3Packages.pyserial to your shell") from exc

# Waveshare SC-series protocol. CRITICAL SC-series quirk: 16-bit values use
# the HIGH byte in the FIRST register (of a pair) -- the opposite of the
# ST-series convention. All multi-byte reads/writes below follow that order.
INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03

ADDR_TORQUE_ENABLE = 40
ADDR_GOAL = 42          # 6-byte block: position(H,L), TIME(H,L), speed(H,L)
ADDR_PRESENT_POSITION = 56

POS_MAX = 1023          # SC15: 180 deg / 1024 counts
POS_MIDDLE = 511


class SC15Bus:
    """Minimal half-duplex UART driver for a Waveshare SC15 bus servo."""

    def __init__(self, port, baud, servo_id, timeout=0.05):
        self.ser = serial.Serial(port, baud, timeout=timeout, write_timeout=timeout)
        self.servo_id = servo_id
        self.timeout = timeout

    def _packet(self, instruction, payload):
        pkt = [0xFF, 0xFF, self.servo_id, len(payload) + 2, instruction] + list(payload)
        pkt.append((~sum(pkt[2:])) & 0xFF)
        return bytes(pkt)

    def _write(self, instruction, payload):
        self.ser.reset_input_buffer()
        self.ser.write(self._packet(instruction, payload))

    def _read_response(self, n_data):
        """Read a valid response frame with `n_data` payload bytes (echo-agnostic)."""
        want = n_data + 6
        header = bytes([0xFF, 0xFF, self.servo_id, n_data + 2])
        deadline = time.monotonic() + self.timeout
        buf = b""
        while time.monotonic() < deadline:
            n = self.ser.in_waiting
            buf += self.ser.read(n if n else 1)
            idx = buf.rfind(header)
            if idx >= 0 and len(buf) >= idx + want:
                resp = buf[idx:idx + want]
                if ((~sum(resp[2:-1])) & 0xFF) == resp[-1]:
                    return resp
        return None

    def ping(self):
        self.ser.reset_input_buffer()
        self.ser.write(self._packet(INST_PING, []))
        return self._read_response(0) is not None

    def enable_torque(self, on=True):
        self._write(INST_WRITE, [ADDR_TORQUE_ENABLE, 1 if on else 0])

    def write_pos(self, position, speed):
        """Goal write: 6 bytes at reg 42, HIGH byte first in each pair."""
        position = max(0, min(POS_MAX, int(position)))
        self._write(INST_WRITE, [
            ADDR_GOAL,
            (position >> 8) & 0xFF, position & 0xFF,     # position (H, L)
            0x00, 0x00,                                 # TIME (0 = use speed)
            (speed >> 8) & 0xFF, speed & 0xFF,          # speed (H, L)
        ])

    def read_position(self):
        self.ser.reset_input_buffer()
        self.ser.write(self._packet(INST_READ, [ADDR_PRESENT_POSITION, 2]))
        resp = self._read_response(2)
        if resp is None:
            return None
        return (resp[5] << 8) | resp[6]                 # HIGH byte first

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


class SC15Driver(Node):
    """Rocks the SC15 between tilt limits and publishes the tilt angle + TF.

    SC15 scale: 180 deg / 1024 counts, middle 511; counts_per_rev = 2048
    (i.e. 1024 counts per 180 deg).
    """

    def __init__(self):
        super().__init__("sc15_driver")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 1000000)
        self.declare_parameter("servo_id", 1)
        self.declare_parameter("center_pos", POS_MIDDLE)
        self.declare_parameter("counts_per_rev", 2048)   # 1024 per 180 deg
        self.declare_parameter("min_angle_deg", -30.0)
        self.declare_parameter("max_angle_deg", 30.0)
        self.declare_parameter("sweep_period", 8.0)
        self.declare_parameter("profile", "sine")      # "sine" or "triangle"
        self.declare_parameter("speed", 400)            # steps/s
        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("readback", True)
        self.declare_parameter("parent_frame", "lidar_mount")
        self.declare_parameter("child_frame", "laser")

        p = lambda n: self.get_parameter(n).value

        self.center = int(p("center_pos"))
        self.cpr = float(p("counts_per_rev"))
        self.min_deg = float(p("min_angle_deg"))
        self.max_deg = float(p("max_angle_deg"))
        self.period = max(0.5, float(p("sweep_period")))
        self.profile = str(p("profile"))
        self.speed = int(p("speed"))
        self.readback = bool(p("readback"))
        self.parent_frame = str(p("parent_frame"))
        self.child_frame = str(p("child_frame"))

        self.bus = SC15Bus(str(p("port")), int(p("baud_rate")), int(p("servo_id")))
        if self.bus.ping():
            pos = self.bus.read_position()
            pos_note = f", position {pos}" if pos is not None else ""
            self.get_logger().info(
                f"SC15 (id {int(p('servo_id'))}) responding on {p('port')} "
                f"@ {int(p('baud_rate'))} baud{pos_note}")
            self.bus.enable_torque(True)
        else:
            self.get_logger().warn(
                "SC15 did not answer PING: check port / baud_rate / servo_id / "
                "wiring. Continuing open-loop.")

        self.tilt_pub = self.create_publisher(Float64, "~/tilt", 10)
        self.tf_pub = TransformBroadcaster(self)
        self.lock = threading.Lock()
        self.t0 = self.get_clock().now()
        self._err_logged = False
        self.timer = self.create_timer(1.0 / float(p("control_rate")), self.cycle)

    def target_angle_deg(self, t):
        lo, hi = self.min_deg, self.max_deg
        if self.profile == "triangle":
            f = (t % self.period) / self.period
            return lo + (1.0 - abs(2.0 * f - 1.0)) * (hi - lo)
        return 0.5 * (lo + hi) + 0.5 * (hi - lo) * math.sin(2.0 * math.pi * t / self.period)

    def cycle(self):
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        deg = self.target_angle_deg(t)
        pos = self.center + int(round(deg / 360.0 * self.cpr))
        actual_deg = deg

        try:
            with self.lock:
                self.bus.write_pos(pos, self.speed)
                if self.readback:
                    rp = self.bus.read_position()
                    if rp is not None and 0 <= rp <= POS_MAX:
                        actual_deg = (rp - self.center) / self.cpr * 360.0
            self._err_logged = False
        except serial.SerialException as e:
            if not self._err_logged:
                self.get_logger().error(f"SC15 serial error: {e}")
                self._err_logged = True
            return

        rad = math.radians(actual_deg)

        msg = Float64()
        msg.data = rad
        self.tilt_pub.publish(msg)

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.parent_frame
        tf.child_frame_id = self.child_frame
        tf.transform.rotation.y = math.sin(rad / 2.0)
        tf.transform.rotation.w = math.cos(rad / 2.0)
        self.tf_pub.sendTransform(tf)

    def destroy_node(self):
        try:
            with self.lock:
                self.bus.enable_torque(False)
                self.bus.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SC15Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
