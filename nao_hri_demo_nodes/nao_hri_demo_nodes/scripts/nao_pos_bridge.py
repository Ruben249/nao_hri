#!/usr/bin/env python3
import json
import time
from typing import Any, Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.action import ActionClient

from nao_pos_interfaces.action import PosPlay


def _safe_json(raw: str) -> Dict[str, Any]:
    # Accept plain strings or JSON objects; if JSON parsing fails, return the raw string as "cmd"
    s = (raw or "").strip()
    if not s:
        return {}
    if not s.startswith("{"):
        return {"cmd": s}
    try:
        obj = json.loads(s)
        return obj if isinstance(obj, dict) else {"cmd": s}
    except Exception:
        return {"cmd": s}


def _strip_pos_ext(name: str) -> str:
    n = (name or "").strip()
    # Normalize names by stripping .pos extension if present, to allow both "rightarmUp" and "rightarmUp.pos"
    while n.lower().endswith(".pos"):
        n = n[:-4]
    return n


class NaoPosBridge(Node):
    """
    Bridge BT -> nao_pos_action (nao_pos_server)
    IN : /hri/nao_pos_cmd  (String)  e.g. "rightarmUp.pos"  o JSON {"pos_file":"rightarmUp.pos"}
    OUT: /hri/nao_pos_result (String) "running" (1 time) and next "success" or "failure" (1 time)
    """

    def __init__(self):
        super().__init__("nao_pos_bridge")

        self.declare_parameter("action_name", "nao_pos_action")
        self.declare_parameter("cmd_topic", "/hri/nao_pos_cmd")
        self.declare_parameter("result_topic", "/hri/nao_pos_result")
        self.declare_parameter("wait_server_s", 2.0)

        self.action_name = str(self.get_parameter("action_name").value)
        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.result_topic = str(self.get_parameter("result_topic").value)
        self.wait_server_s = float(self.get_parameter("wait_server_s").value)

        self.pub = self.create_publisher(String, self.result_topic, 10)
        self.sub = self.create_subscription(String, self.cmd_topic, self._on_cmd, 10)

        self.client = ActionClient(self, PosPlay, self.action_name)

        self._seq = 0
        self._active = False

        self.get_logger().info(f"nao_pos_bridge ready. action={self.action_name}")
        self.get_logger().info(f"Subscribed: {self.cmd_topic} | Publishing: {self.result_topic}")

    def _pub_status(self, s: str):
        m = String()
        m.data = s
        self.pub.publish(m)

    def _cancel_active(self):
        # cancel best-effort; though we don't keep the goal handle, the server should handle preemption if a new goal is sent.
        if not self._active:
            return
        try:
            # Don't have the goal handle, so we can't explicitly cancel; we rely on the server's behavior of preempting old goals when new ones arrive.
            pass
        except Exception:
            pass

    def _extract_action_name(self, data: Dict[str, Any]) -> str:
        raw = (
            data.get("action_name")
            or data.get("pos_file")
            or data.get("pose_file")
            or data.get("file")
            or data.get("pose")
            or data.get("cmd")
            or ""
        )
        raw = (raw or "").strip()
        return _strip_pos_ext(raw)

    def _on_cmd(self, msg: String):
        data = _safe_json(msg.data)
        action_name = self._extract_action_name(data)

        if not action_name:
            self.get_logger().warn("nao_pos_cmd vacío -> failure")
            self._pub_status("failure")
            return

        # New command: Increment sequence to invalidate old callbacks, and mark active
        self._seq += 1
        seq = self._seq
        self._active = True

        self._cancel_active()

        # Wait for server with timeout; if not available, publish failure and return
        if not self.client.wait_for_server(timeout_sec=self.wait_server_s):
            self.get_logger().error(f"Action server no disponible: {self.action_name}")
            if seq == self._seq:
                self._active = False
                self._pub_status("failure")
            return

        goal = PosPlay.Goal()
        goal.action_name = action_name

        self.get_logger().info(f"Sending goal: action_name={action_name}")

        send_future = self.client.send_goal_async(goal)

        # Callback for when the goal response is received (accepted/rejected)
        def _goal_response_cb(fut):
            if seq != self._seq:
                return  #old callback, ignore
            try:
                gh = fut.result()
            except Exception as e:
                self.get_logger().error(f"send_goal_async error: {e}")
                if seq == self._seq:
                    self._active = False
                    self._pub_status("failure")
                return

            # Check if goal was accepted
            if not gh.accepted:
                self.get_logger().warn("goal rejected")
                if seq == self._seq:
                    self._active = False
                    self._pub_status("failure")
                return

            if seq == self._seq:
                self._pub_status("running")

            res_future = gh.get_result_async()

            # Callback for when the result is received
            def _result_cb(rf):
                if seq != self._seq:
                    return
                try:
                    wrapped = rf.result()
                except Exception as e:
                    self.get_logger().error(f"get_result_async error: {e}")
                    if seq == self._seq:
                        self._active = False
                        self._pub_status("failure")
                    return

                # The actual result message is wrapped in the ActionResult structure; we need to extract it and check the 'success' field.
                result_msg = getattr(wrapped, "result", None)
                success = False
                # We check if the result message has a 'success' attribute and try to interpret it as a boolean. 
                # If it's not present or cannot be interpreted, we default to False.
                if result_msg is not None and hasattr(result_msg, "success"):
                    try:
                        success = bool(result_msg.success)
                    except Exception:
                        success = False

                if seq == self._seq:
                    self._active = False
                    time.sleep(3)
                    self._pub_status("success" if success else "failure")

            res_future.add_done_callback(_result_cb)

        send_future.add_done_callback(_goal_response_cb)


def main():
    rclpy.init()
    node = NaoPosBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
