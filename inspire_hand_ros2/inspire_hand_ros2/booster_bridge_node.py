"""Same command topics as the driver, forwarded to the Booster T1's own hand control.

Use this instead of the driver when the hand is plugged into the T1's hand port and
the robot reports it (B1LocoClient::GetHands). The robot then owns the RS485 link,
so this node only sends ControlDexterousHand requests through Booster's
/booster_rpc_service (needs the booster_interface package from
booster_robotics_sdk_ros2). It publishes no feedback; the robot publishes hand
data on its own DDS topic rt/booster_hand_data.
"""
import json
import time

import rclpy

from . import rh56
from .driver_node import HandCommandNode

CONTROL_DEXTEROUS_HAND = 2013  # LocoApiId::kControlDexterousHand (booster_robotics_sdk b1_loco_api.hpp)
INSPIRE_HAND = 0               # BoosterHandType::kInspireHand -- RH56DFX has no tactile arrays


class BoosterHandBridge(HandCommandNode):
    def __init__(self):
        super().__init__("inspire_hand_bridge")
        from booster_interface.srv import RpcService  # only needed for this node

        self.RpcService = RpcService
        self.hand_index = self.declare_parameter("hand_index", 0).value  # 0 = left, 1 = right
        rate = self.declare_parameter("rate_hz", 20.0).value
        self.client = self.create_client(RpcService, "/booster_rpc_service")
        self.last_angles = [rh56.HOLD] * 6
        self.sent_at = None
        self.create_timer(1.0 / rate, self._tick)

    def _tick(self):
        if self.speed_dirty or self.force_dirty:
            # Booster sends speed/force with each finger target, so re-send the last targets.
            self.speed_dirty = self.force_dirty = False
            self._queue_angles(list(self.last_angles))
        if self.pending_angles is None:
            return
        if self.sent_at is not None and time.monotonic() - self.sent_at < 1.0:
            return  # previous request still in flight
        if not self.client.service_is_ready():
            self.get_logger().warn("/booster_rpc_service not available", throttle_duration_sec=5.0)
            return
        angles, self.pending_angles = self.pending_angles, None
        fingers = [{"seq": i, "angle": a, "force": self.force[i], "speed": self.speed[i]}
                   for i, a in enumerate(angles) if a != rh56.HOLD]
        if not fingers:
            return
        self.last_angles = [a if a != rh56.HOLD else old for a, old in zip(angles, self.last_angles)]
        req = self.RpcService.Request()
        req.msg.api_id = CONTROL_DEXTEROUS_HAND
        req.msg.body = json.dumps({"finger_params": fingers, "hand_index": self.hand_index,
                                   "hand_type": INSPIRE_HAND})
        self.sent_at = time.monotonic()
        self.client.call_async(req).add_done_callback(self._done)

    def _done(self, future):
        self.sent_at = None
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().warn("ControlDexterousHand failed: %s" % e)
            return
        if resp.msg.status != 0:
            self.get_logger().warn("ControlDexterousHand refused (status %d): %s -- is the hand listed "
                                   "by GetHands and the robot out of damping mode?"
                                   % (resp.msg.status, resp.msg.body), throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = BoosterHandBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
