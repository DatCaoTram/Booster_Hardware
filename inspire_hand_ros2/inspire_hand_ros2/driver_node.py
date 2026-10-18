"""ROS2 driver for an Inspire RH56 hand on a USB-RS485 adapter.

Topics are relative, so launch it in a namespace (e.g. /left_hand):

  sub  cmd_angle    std_msgs/Int16MultiArray  6 raw targets, 0 (closed)..1000 (open), -1 = hold
  sub  cmd_joint    sensor_msgs/JointState    targets in radians, by joint name (partial is fine)
  sub  cmd_speed    std_msgs/Int16MultiArray  6 x 0..1000
  sub  cmd_force    std_msgs/Int16MultiArray  6 x force limit in grams (thumb up to 1500)
  pub  joint_states sensor_msgs/JointState    position in radians, effort in N
  pub  angle_raw    std_msgs/Int16MultiArray  6 x 0..1000
  pub  force_raw    std_msgs/Int16MultiArray  6 x grams
  pub  /diagnostics diagnostic_msgs/DiagnosticArray  status, errors, current, temperature
  srv  clear_error  std_srvs/Trigger
"""
import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16MultiArray
from std_srvs.srv import Trigger

from . import rh56
from .sim import SimulatedHand

# Flexion from fully open, from the manual's ranges: fingers 19..176.7 deg,
# thumb bend -13..53.6 deg, thumb rotation 90..165 deg. Override to match your URDF.
DEFAULT_CLOSED_RAD = [2.752, 2.752, 2.752, 2.752, 1.162, 1.309]


class HandCommandNode(Node):
    """Command topics and joint mapping shared by the driver and the Booster bridge."""

    def __init__(self, name):
        super().__init__(name)
        prefix = self.declare_parameter("joint_prefix", "left_").value
        self.speed = rh56.clamp_speeds(self.declare_parameter("speed", [1000] * 6).value)
        self.force = rh56.clamp_forces(self.declare_parameter("force", [500] * 6).value)
        self.open_rad = list(self.declare_parameter("joint_open_rad", [0.0] * 6).value)
        self.closed_rad = list(self.declare_parameter("joint_closed_rad", DEFAULT_CLOSED_RAD).value)
        self.joint_names = [prefix + n for n in rh56.DOF_NAMES]
        self.pending_angles = None
        self.speed_dirty = self.force_dirty = True

        self.create_subscription(Int16MultiArray, "cmd_angle", self._on_cmd_angle, 10)
        self.create_subscription(JointState, "cmd_joint", self._on_cmd_joint, 10)
        self.create_subscription(Int16MultiArray, "cmd_speed", self._on_cmd_speed, 10)
        self.create_subscription(Int16MultiArray, "cmd_force", self._on_cmd_force, 10)

    def raw_to_rad(self, i, raw):
        return self.closed_rad[i] + raw / 1000.0 * (self.open_rad[i] - self.closed_rad[i])

    def rad_to_raw(self, i, rad):
        if math.isnan(rad):
            return rh56.HOLD
        raw = (rad - self.closed_rad[i]) / (self.open_rad[i] - self.closed_rad[i]) * 1000.0
        return max(0, min(1000, int(round(raw))))

    def _six(self, msg, topic):
        if len(msg.data) != 6:
            self.get_logger().warn("%s needs 6 values, got %d" % (topic, len(msg.data)))
            return None
        return list(msg.data)

    def _queue_angles(self, angles):
        # Commands faster than the bus rate are merged; the newest value per DOF wins.
        if self.pending_angles is None:
            self.pending_angles = angles
        else:
            self.pending_angles = [p if n == rh56.HOLD else n for n, p in zip(angles, self.pending_angles)]

    def _on_cmd_angle(self, msg):
        angles = self._six(msg, "cmd_angle")
        if angles is not None:
            self._queue_angles(rh56.clamp_angles(angles))

    def _on_cmd_joint(self, msg):
        if msg.name:
            angles = [rh56.HOLD] * 6
            for name, pos in zip(msg.name, msg.position):
                if name in self.joint_names:
                    i = self.joint_names.index(name)
                    angles[i] = self.rad_to_raw(i, pos)
                else:
                    self.get_logger().warn("cmd_joint: unknown joint %r" % name, throttle_duration_sec=5.0)
        elif len(msg.position) == 6:
            angles = [self.rad_to_raw(i, p) for i, p in enumerate(msg.position)]
        else:
            self.get_logger().warn("cmd_joint needs joint names or 6 positions")
            return
        self._queue_angles(angles)

    def _on_cmd_speed(self, msg):
        speed = self._six(msg, "cmd_speed")
        if speed is not None:
            self.speed, self.speed_dirty = rh56.clamp_speeds(speed), True

    def _on_cmd_force(self, msg):
        force = self._six(msg, "cmd_force")
        if force is not None:
            self.force, self.force_dirty = rh56.clamp_forces(force), True


class HandDriver(HandCommandNode):
    def __init__(self):
        super().__init__("inspire_hand")
        self.device = self.declare_parameter("port", "/dev/ttyUSB0").value
        self.baudrate = self.declare_parameter("baudrate", 115200).value
        self.hand_id = self.declare_parameter("hand_id", 1).value
        self.simulate = self.declare_parameter("simulate", False).value
        rate = self.declare_parameter("rate_hz", 50.0).value

        self.hand = None
        self.state = None
        self.fault = "not connected"
        self._next_connect = 0.0

        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.angle_pub = self.create_publisher(Int16MultiArray, "angle_raw", 10)
        self.force_pub = self.create_publisher(Int16MultiArray, "force_raw", 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_service(Trigger, "clear_error", self._on_clear_error)
        self.create_timer(1.0 / rate, self._tick)
        self.create_timer(0.5, self._publish_diagnostics)

    def _connect(self):
        now = time.monotonic()
        if now < self._next_connect:
            return False
        self._next_connect = now + 1.0
        try:
            if self.simulate:
                self.hand = rh56.RH56(SimulatedHand(self.hand_id), self.hand_id)
            else:
                self.hand = rh56.RH56.open(self.device, self.hand_id, self.baudrate)
        except Exception as e:  # SerialException, missing pyserial, permissions...
            self.fault = "cannot open %s: %s" % (self.device, e)
            self.get_logger().error(self.fault, throttle_duration_sec=5.0)
            return False
        self.speed_dirty = self.force_dirty = True
        self.get_logger().info("opened %s, hand id %d" % ("simulator" if self.simulate else self.device,
                                                          self.hand_id))
        return True

    def _tick(self):
        if self.hand is None and not self._connect():
            return
        try:
            if self.speed_dirty:
                self.hand.set_speeds(self.speed)
                self.speed_dirty = False
            if self.force_dirty:
                self.hand.set_forces(self.force)
                self.force_dirty = False
            if self.pending_angles is not None:
                self.hand.set_angles(self.pending_angles)
                self.pending_angles = None
            angle = self.hand.read_angles()
            feedback = self.hand.read_feedback()
        except rh56.ProtocolError as e:
            self.fault = str(e)
            self.get_logger().warn(self.fault, throttle_duration_sec=2.0)
            return
        except OSError as e:  # adapter unplugged etc. -- reopen on the next tick
            self.fault = "serial error: %s" % e
            self.get_logger().error(self.fault)
            try:
                self.hand.close()
            except OSError:
                pass
            self.hand = None
            return

        self.fault = None
        self.state = dict(feedback, angle=angle)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = [self.raw_to_rad(i, a) for i, a in enumerate(angle)]
        js.effort = [f * 9.81e-3 for f in feedback["force"]]
        self.joint_pub.publish(js)
        self.angle_pub.publish(Int16MultiArray(data=angle))
        self.force_pub.publish(Int16MultiArray(data=feedback["force"]))

    def _publish_diagnostics(self):
        st = DiagnosticStatus(name=self.get_fully_qualified_name(),
                              hardware_id="RH56 id %d on %s" % (self.hand_id, self.device))
        if self.fault or self.state is None:
            st.level, st.message = DiagnosticStatus.ERROR, self.fault or "no data"
        else:
            s = self.state
            level, st.message = rh56.summarize(s["status"], s["error"])
            st.level = (DiagnosticStatus.OK, DiagnosticStatus.WARN, DiagnosticStatus.ERROR)[level]
            for i, name in enumerate(rh56.DOF_NAMES):
                st.values.append(KeyValue(key=name, value="angle %d, force %d g, %d mA, %d C, %s%s" % (
                    s["angle"][i], s["force"][i], s["current"][i], s["temp"][i],
                    rh56.STATUS_TEXT.get(s["status"][i], "status %d" % s["status"][i]),
                    ", ERROR: " + rh56.error_text(s["error"][i]) if s["error"][i] else "")))
        msg = DiagnosticArray(status=[st])
        msg.header.stamp = self.get_clock().now().to_msg()
        self.diag_pub.publish(msg)

    def _on_clear_error(self, request, response):
        try:
            if self.hand is None:
                raise rh56.ProtocolError(self.fault)
            self.hand.clear_errors()
            response.success, response.message = True, "errors cleared"
        except (rh56.ProtocolError, OSError) as e:
            response.success, response.message = False, str(e)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = HandDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
