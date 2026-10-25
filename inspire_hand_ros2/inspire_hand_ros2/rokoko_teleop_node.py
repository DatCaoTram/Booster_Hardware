"""Rokoko Smartglove -> RH56: listens to Rokoko Studio's live stream and publishes cmd_angle.

Run it in the hand's namespace so it feeds the driver (or the Booster bridge), and point Rokoko
Studio's Custom stream at this machine's IP, UDP port 14043:

  ros2 launch inspire_hand_ros2 hand.launch.py port:=... rokoko:=true

  pub  cmd_angle         std_msgs/Int16MultiArray
  srv  calibrate_open    std_srvs/Trigger   glove hand flat, fingers straight
  srv  calibrate_closed  std_srvs/Trigger   fist, thumb folded across the fingers
  srv  enable            std_srvs/SetBool   pause / resume following the glove
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from std_srvs.srv import SetBool, Trigger

from . import rokoko


class RokokoTeleop(Node):
    def __init__(self):
        super().__init__("rokoko_teleop")
        param = lambda name, default: self.declare_parameter(name, default).value
        self.teleop = rokoko.GloveTeleop(
            side=param("glove_side", "left"),
            port=param("udp_port", rokoko.DEFAULT_PORT),
            actor=param("actor", ""),
            calibration_file=param("calibration_file", rokoko.DEFAULT_CALIBRATION),
            smoothing=param("smoothing", 0.5))
        self.deadband = param("deadband", 5)
        self.enabled = param("enabled", True)
        self.sent = None
        self.last_status = None

        self.pub = self.create_publisher(Int16MultiArray, "cmd_angle", 10)
        self.create_service(Trigger, "calibrate_open", lambda req, res: self._calibrate("open", res))
        self.create_service(Trigger, "calibrate_closed", lambda req, res: self._calibrate("closed", res))
        self.create_service(SetBool, "enable", self._on_enable)
        self.create_timer(1.0 / param("rate_hz", 30.0), self._tick)
        self.create_timer(1.0, self._log_status)

    def _tick(self):
        target = self.teleop.update()
        if target is None or not self.enabled:
            return
        if self.sent is None or max(abs(a - b) for a, b in zip(target, self.sent)) >= self.deadband:
            self.pub.publish(Int16MultiArray(data=target))
            self.sent = target

    def _log_status(self):
        status = self.teleop.status()
        if status != self.last_status:
            self.get_logger().info(status)
            self.last_status = status

    def _calibrate(self, pose, response):
        try:
            response.success, response.message = True, self.teleop.calibrate(pose)
        except RuntimeError as e:
            response.success, response.message = False, str(e)
        return response

    def _on_enable(self, request, response):
        self.enabled, self.sent = request.data, None
        response.success = True
        response.message = "following the glove" if self.enabled else "paused"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RokokoTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.teleop.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
