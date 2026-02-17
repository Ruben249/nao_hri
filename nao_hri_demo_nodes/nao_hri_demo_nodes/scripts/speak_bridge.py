#!/usr/bin/env python3
import json
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from hni_interfaces.srv import TextToSpeech


def _safe_json(raw: str) -> dict:
    """
    Accept plain strings or JSON objects; if JSON parsing fails, return the raw string as "text".
    """
    s = (raw or "").strip()
    if not s:
        return {}
    if not s.startswith("{"):
        return {"text": s}
    try:
        obj = json.loads(s)
        return obj if isinstance(obj, dict) else {"text": s}
    except Exception:
        return {"text": s}


class SpeakBridge(Node):
    """
    BT-compatible bridge:
      - Sub: /hri/speak_cmd    (std_msgs/String)  -> texto o JSON {"text": "..."}
      - Pub: /hri/speak_result (std_msgs/String)  -> running/success/failure
    """

    def __init__(self):
        super().__init__("speak_bridge")

        self.declare_parameter("cmd_topic", "/hri/speak_cmd")
        self.declare_parameter("result_topic", "/hri/speak_result")
        self.declare_parameter("service_name", "gtts_service")
        self.declare_parameter("timeout_sec", 10.0)

        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.result_topic = str(self.get_parameter("result_topic").value)
        self.service_name = str(self.get_parameter("service_name").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)

        self.pub_result = self.create_publisher(String, self.result_topic, 10)
        self.sub_cmd = self.create_subscription(String, self.cmd_topic, self._on_cmd, 10)

        self.cli = self.create_client(TextToSpeech, self.service_name)

        self._token = 0
        self._lock = threading.Lock()
        self._active_timer: Optional[rclpy.timer.Timer] = None

        self.get_logger().info("speak_bridge ready")
        self.get_logger().info(f" Subscribed: {self.cmd_topic}")
        self.get_logger().info(f" Publishing:  {self.result_topic}")
        self.get_logger().info(f" Service:     {self.service_name}")
        self.get_logger().info(f" Timeout:     {self.timeout_sec}s")

    def _pub_status(self, status: str):
        # Publish status as a String message
        msg = String()
        msg.data = status
        self.pub_result.publish(msg)

    def _cancel_timeout_timer_locked(self):
        # This should be called with self._lock held. It cancels any active timeout timer.
        if self._active_timer is not None:
            try:
                self._active_timer.cancel()
            except Exception:
                pass
            self._active_timer = None

    def _start_timeout_timer(self, token: int):
        """
        This starts a timer that will check if the TTS operation has timed out. 
        If the timer expires and the token is still active, it will publish a failure status.
        """
        def _on_timeout():
            with self._lock:
                if token != self._token:
                    return
                self._cancel_timeout_timer_locked()
            self.get_logger().error("TTS timed out")
            self._pub_status("failure")

        # We start the timer with the specified timeout duration. If the timer expires, it will call _on_timeout.
        with self._lock:
            self._cancel_timeout_timer_locked()
            self._active_timer = self.create_timer(self.timeout_sec, _on_timeout)

    def _on_cmd(self, msg: String):
        # The incoming message can be a plain string or a JSON object. We try to parse it as JSON, but if that fails, we treat it as plain text.
        raw = (msg.data or "").strip()

        with self._lock:
            self._token += 1
            token = self._token
            self._cancel_timeout_timer_locked()

        if not raw:
            self._pub_status("failure")
            return

        data = _safe_json(raw)
        text = (data.get("text") or "").strip()
        if not text:
            self._pub_status("failure")
            return

        # Wait for the TTS service to be available; if it's not available within a short timeout, publish failure and return
        if not self.cli.wait_for_service(timeout_sec=0.2):
            self.get_logger().error("gtts_service not available")
            self._pub_status("failure")
            return

        req = TextToSpeech.Request()
        req.text = text

        self.get_logger().info(f"cmd received: text_len={len(text)} token={token}")
        self._pub_status("running")
        self._start_timeout_timer(token)

        future = self.cli.call_async(req)

        def _done_cb(fut):
            """
            This callback is called when the TTS service call completes (either successfully or with an error).
            We check if the token is still active, and if so, we process the result. If the service call was successful and indicates success, 
            we publish "success"; otherwise, we publish "failure"
            """
            with self._lock:
                if token != self._token:
                    return
                self._cancel_timeout_timer_locked()

            try:
                resp = fut.result()
            except Exception as e:
                self.get_logger().error(f"TTS service call failed: {e}")
                self._pub_status("failure")
                return

            if getattr(resp, "success", False):
                self._pub_status("success")
            else:
                dbg = getattr(resp, "debug", "")
                if dbg:
                    self.get_logger().error(f"TTS failure: {dbg}")
                self._pub_status("failure")

        future.add_done_callback(_done_cb)


def main():
    rclpy.init()
    node = SpeakBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
