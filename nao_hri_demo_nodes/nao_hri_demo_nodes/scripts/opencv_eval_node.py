#!/usr/bin/env python3
import os
import json
import time
import math
from dataclasses import dataclass
from typing import Dict, Any, Optional, Tuple, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Image

import yaml
import cv2
from cv_bridge import CvBridge
import mediapipe as mp

DEFAULT_ANGLE_TOL_DEG = 30.0
DEFAULT_MIN_KPT_CONF = 0.35

RESET_HITS_ON_FAIL = True
TRACE_ANALYZING_EVERY_FRAME = True

ANALYZING_TRACE_PERIOD_S = 0.25

IGNORE_EVAL_CMD_WHILE_ACTIVE = True
EVAL_CMD_DEBOUNCE_S = 0.25


def _safe_json(s: str) -> Dict[str, Any]:
    # Accept plain strings or JSON objects; if JSON parsing fails, return the raw string as "cmd"
    s = (s or "").strip()
    if not s:
        return {}
    if s[0] != "{":
        return {"cmd": s}
    try:
        return json.loads(s)
    except Exception:
        return {"cmd": s}


def clamp(x: float, a: float, b: float) -> float:
    return max(a, min(b, x))


def angle_deg(a: Tuple[float, float], b: Tuple[float, float], c: Tuple[float, float]) -> float:
    """
    ABC angle in degrees. 0° = a-b-c aligned, 180° = b is "inside" the angle formed by a and c.
    """
    bax = a[0] - b[0]
    bay = a[1] - b[1]
    bcx = c[0] - b[0]
    bcy = c[1] - b[1]
    nba = math.hypot(bax, bay)
    nbc = math.hypot(bcx, bcy)
    if nba < 1e-6 or nbc < 1e-6:
        return 0.0
    cosang = (bax * bcx + bay * bcy) / (nba * nbc)
    cosang = clamp(cosang, -1.0, 1.0)
    return math.degrees(math.acos(cosang))


def angle_to_vertical_deg(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    """
    Angle to vertical in degrees. 0° = vertical (b above a), 90° = horizontal, 180° = b below a.
    """
    vx = b[0] - a[0]
    vy = b[1] - a[1]
    nv = math.hypot(vx, vy)
    if nv < 1e-6:
        return 180.0
    vx /= nv
    vy /= nv
    cosang = clamp(-vy, -1.0, 1.0)  # dot((vx,vy),(0,-1)) = -vy
    return math.degrees(math.acos(cosang))


class PoseEstimator:
    """
    MediaPipe Pose.
    Landmarks used in templates: nose, left_shoulder, right_shoulder, left_elbow, right_elbow,
      left_wrist, right_wrist, left_hip, right_hip.
    """
    def __init__(self, logger):
        self._logger = logger
        self._mp_pose = mp.solutions.pose
        self._pose = self._mp_pose.Pose(
            static_image_mode=False,
            model_complexity=1,              # CPU-friendly
            enable_segmentation=False,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self._logger.info("PoseEstimator: usando MediaPipe Pose (CPU).")

        self._name_to_idx = {
            "nose": 0,
            "left_shoulder": 11,
            "right_shoulder": 12,
            "left_elbow": 13,
            "right_elbow": 14,
            "left_wrist": 15,
            "right_wrist": 16,
            "left_hip": 23,
            "right_hip": 24,
        }

    def estimate(self, bgr_img) -> Optional[Dict[str, Tuple[float, float, float]]]:
        # Convert BGR to RGB as MediaPipe expects RGB images. The output is a dict mapping landmark names to
        #  (x, y, visibility).
        rgb = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)
        res = self._pose.process(rgb)
        if not res.pose_landmarks:
            return None
        lm = res.pose_landmarks.landmark

        out: Dict[str, Tuple[float, float, float]] = {}
        for name, idx in self._name_to_idx.items():
            p = lm[idx]
            out[name] = (float(p.x), float(p.y), float(p.visibility))
        return out


@dataclass
# Class to hold the configuration for a pose template, loaded from a YAML file. 
# It includes the ID of the template, the number of consecutive hits required,
class Check:
    type: str
    name: str
    a: Optional[str] = None
    b: Optional[str] = None
    c: Optional[str] = None
    target_deg: Optional[float] = None
    tol_deg: Optional[float] = None
    margin_norm: Optional[float] = None


@dataclass
# Class to hold the configuration for a pose template, loaded from a YAML file. 
# It includes the ID of the template, the number of consecutive hits required,
class TemplateCfg:
    id: str
    consecutive_hits: int
    min_kpt_conf: float
    required_landmarks: List[str]
    logic: str  # all | any
    checks: List[Check]


def load_template(yml_path: str) -> TemplateCfg:
    """
    Loads a pose template configuration from a YAML file. The YAML file should define the structure of the pose 
    to be evaluated, including the required landmarks, the checks to perform (e.g., angles between landmarks), 
    and other parameters. The function returns a TemplateCfg object that encapsulates this configuration.
    """
    with open(yml_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    tid = str(data.get("id", "")).strip() or os.path.splitext(os.path.basename(yml_path))[0]
    consecutive_hits = max(1, int(data.get("consecutive_hits", 1)))
    min_kpt_conf = float(data.get("min_kpt_conf", DEFAULT_MIN_KPT_CONF))

    required = data.get("required_landmarks", []) or []
    required = [str(x).strip() for x in required if str(x).strip()]

    logic = str(data.get("logic", "all")).strip().lower()
    if logic not in ("all", "any"):
        logic = "all"

    checks_raw = data.get("checks", []) or []
    checks: List[Check] = []


    """
    Procces the raw checks from the YAML file and converts them into a list of Check objects. 
    Each check defines a specific condition to evaluate on the pose, such as an angle between landmarks or a
     positional relationship. The function handles different types of checks and extracts the necessary parameters 
    for each check type.
    """
    for c in checks_raw:
        if not isinstance(c, dict):
            continue
        ctype = str(c.get("type", "")).strip()
        name = str(c.get("name", ctype)).strip()

        a = c.get("a", None)
        b = c.get("b", None)
        cc = c.get("c", None)

        target = c.get("target_deg", None)
        tol = c.get("tol_deg", None)
        margin = c.get("margin_norm", None)

        checks.append(Check(
            type=ctype,
            name=name,
            a=str(a).strip() if isinstance(a, str) else None,
            b=str(b).strip() if isinstance(b, str) else None,
            c=str(cc).strip() if isinstance(cc, str) else None,
            target_deg=float(target) if target is not None else None,
            tol_deg=float(tol) if tol is not None else None,
            margin_norm=float(margin) if margin is not None else None,
        ))

    return TemplateCfg(
        id=tid,
        consecutive_hits=consecutive_hits,
        min_kpt_conf=min_kpt_conf,
        required_landmarks=required,
        logic=logic,
        checks=checks,
    )


class OpenCVEvalNode(Node):
    """
    /hri/eval_cmd -> eval de pose con OpenCV/MediaPipe -> /hri/eval_result
    IN: /hri/eval_cmd (String) e.g. "rightarmUp" o JSON {"id":"rightarmUp"}
    OUT: /hri/eval_result (String) "true" o "false"
    The YML templates for the pose should be in the config_dir, named e.g. rightarmUp.yml, with the structure defined
    in TemplateCfg.
    """

    def __init__(self):
        super().__init__("eval_stub") 

        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("config_dir", "")
        self.declare_parameter("timeout_s", 2.0)

        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.config_dir = self.get_parameter("config_dir").get_parameter_value().string_value
        self.timeout_s = float(self.get_parameter("timeout_s").value)

        self.get_logger().info(f"config_dir={self.config_dir}")
        self.get_logger().info(f"eval_stub(OpenCV) listo. image_topic={self.image_topic} timeout_s={self.timeout_s}")

        qos_img = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.bridge = CvBridge()
        self.pose = PoseEstimator(self.get_logger())

        self.sub_img = self.create_subscription(Image, self.image_topic, self._on_img, qos_img)
        self.pub_eval = self.create_publisher(String, "/hri/eval_result", 10)
        self.sub_cmd = self.create_subscription(String, "/hri/eval_cmd", self._on_cmd, 10)

        # Cicle state
        self._armed = False
        self._t0 = 0.0
        self._published = False
        self._hits = 0

        self._active_id = ""
        self._cfg: Optional[TemplateCfg] = None
        self._cfg_path = ""

        self._last_analyzing_log_t = 0.0

        # Avoid spamming eval_result.
        self._last_cmd_time = 0.0
        self._cycle_seq = 0
        self._published_in_cycle = False

    def _publish_once(self, s: str):
        # Safe publish: only publish once per cycle (between eval_cmd and the next eval_cmd). 
        self._published_in_cycle = True

        m = String()
        m.data = s
        self.pub_eval.publish(m)

    def _resolve_cfg_path(self, pose_id: str) -> str:
        # Given a pose ID, this function constructs the expected file paths for the YAML configuration of that pose.
        p1 = os.path.join(self.config_dir, f"{pose_id}.yml")
        p2 = os.path.join(self.config_dir, f"{pose_id}.yaml")
        if os.path.isfile(p1):
            return p1
        if os.path.isfile(p2):
            return p2
        return p1

    def _load_cfg_for_id(self, pose_id: str) -> bool:
        """
        This function attempts to load the configuration for a given pose ID. It constructs the expected file path for the YAML configuration,
        and tries to load it. If successful, it stores the configuration in self._cfg and returns True. If it fails (e.g., file not found or parsing error), it logs an error, sets self._cfg to None, and returns False.
        """
        if not self.config_dir:
            self.get_logger().error("config_dir vacío.")
            self._cfg = None
            return False
        path = self._resolve_cfg_path(pose_id)
        self._cfg_path = path
        try:
            self._cfg = load_template(path)
            return True
        except Exception as e:
            self.get_logger().error(f"No puedo leer template id={pose_id} path={path}: {e}")
            self._cfg = None
            return False

    def _on_cmd(self, msg: String):
        now = time.time()

        # 1) If the last command was received very recently, ignore this command to avoid spamming (debounce).
        if (now - self._last_cmd_time) < EVAL_CMD_DEBOUNCE_S:
            return
        self._last_cmd_time = now

        # 2) If we receive a new command while we are still armed and haven't published a result for the previous command, we can choose to ignore the new command to avoid spamming. This is optional and depends on how you want the system to behave. If you want to allow new commands to reset the evaluation even if we haven't published a result yet, you can skip this check.
        if IGNORE_EVAL_CMD_WHILE_ACTIVE and self._armed and not self._published:
            return

        data = _safe_json(msg.data)
        pose_id = (data.get("id") or "").strip() or "rightarmUp"

        self.get_logger().info(f"eval_cmd recibido: raw={msg.data.strip()} parsed_id={pose_id}")

        # 3) Try to load the configuration for the given pose ID. If it fails, we publish a failure result and return.
        if not self._load_cfg_for_id(pose_id):
            self._armed = False
            self._published = True
            self._cycle_seq += 1
            self._published_in_cycle = False
            self._publish_once("false")
            self.get_logger().info(f"[{pose_id}] RESULT=false (cfg_missing)")
            return

        # Reset cycle state for the new command
        self._cycle_seq += 1
        self._published_in_cycle = False

        self._armed = True
        self._published = False
        self._t0 = now
        self._hits = 0
        self._active_id = pose_id

        self.get_logger().info(f"[{pose_id}] armado. template={self._cfg_path} consec={self._cfg.consecutive_hits}")

    def _on_img(self, msg: Image):
        """
        This callback is called every time a new image is received on the subscribed topic. If we are currently 
        "armed" (i.e., we have received an eval_cmd and are waiting for the pose to be detected), 
        we will process the image to evaluate the pose. If the evaluation criteria are met, we will publish a "true" 
        result; if they are not met within the timeout period, we will publish "false". The function also includes
        logic to avoid spamming results and to log the evaluation process.
        """
        if not self._armed or self._published:
            return

        if (time.time() - self._t0) > self.timeout_s:
            self._armed = False
            self._published = True
            self._publish_once("false")
            self.get_logger().info(f"[{self._active_id}] RESULT=false (timeout)")
            return

        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")
            return

        if TRACE_ANALYZING_EVERY_FRAME:
            now = time.time()
            if (now - self._last_analyzing_log_t) >= ANALYZING_TRACE_PERIOD_S:
                self._last_analyzing_log_t = now
                self.get_logger().info(f"[{self._active_id}] Analizando...")

        ok, debug = self._evaluate_frame(img)
        self.get_logger().info(f"[{self._active_id}] match={ok} {debug}")

        if ok:
            self._hits += 1
            if self._cfg is not None and self._hits >= self._cfg.consecutive_hits:
                self._armed = False
                self._published = True
                self._publish_once("true")
                self.get_logger().info(f"[{self._active_id}] RESULT=true")
        else:
            if RESET_HITS_ON_FAIL:
                self._hits = 0

    def _evaluate_frame(self, bgr_img) -> Tuple[bool, str]:
        """
        Evaluate the given image against the currently active pose template configuration. This function extracts 
        the keypoints from the image using MediaPipe Pose, checks the required landmarks and their confidence,
        and then performs the specified checks (e.g., angles between landmarks). It returns a boolean indicating 
        whether the pose matches the template, and a debug string with details about the evaluation.
        """
        if self._cfg is None:
            return False, "(no_cfg)"

        kpts = self.pose.estimate(bgr_img)
        if kpts is None:
            return False, "(no_pose)"

        min_vis = self._cfg.min_kpt_conf
        for name in self._cfg.required_landmarks:
            if name not in kpts:
                return False, f"(missing_landmark {name})"
            if kpts[name][2] < min_vis:
                return False, f"(low_vis {name}={kpts[name][2]:.2f} min={min_vis:.2f})"

        results: List[bool] = []
        parts: List[str] = []

        # Perform the checks defined in the template configuration. Each check is evaluated based on its type and 
        # parameters, and the results are collected to determine the final evaluation outcome.
        for chk in self._cfg.checks:
            if chk.type == "angle":
                if not (chk.a and chk.b and chk.c):
                    results.append(False); parts.append(f"{chk.name}=BAD_ARGS"); continue
                if chk.a not in kpts or chk.b not in kpts or chk.c not in kpts:
                    results.append(False); parts.append(f"{chk.name}=MISSING"); continue

                val = angle_deg(
                    (kpts[chk.a][0], kpts[chk.a][1]),
                    (kpts[chk.b][0], kpts[chk.b][1]),
                    (kpts[chk.c][0], kpts[chk.c][1]),
                )
                tgt = chk.target_deg if chk.target_deg is not None else 0.0
                tol = chk.tol_deg if chk.tol_deg is not None else DEFAULT_ANGLE_TOL_DEG
                ok = abs(val - tgt) <= tol

                results.append(ok)
                parts.append(f"{chk.name}={val:.1f} tgt={tgt:.1f} tol={tol:.1f} ok={ok}")

            elif chk.type == "angle_to_vertical":
                if not (chk.a and chk.b):
                    results.append(False); parts.append(f"{chk.name}=BAD_ARGS"); continue
                if chk.a not in kpts or chk.b not in kpts:
                    results.append(False); parts.append(f"{chk.name}=MISSING"); continue

                val = angle_to_vertical_deg(
                    (kpts[chk.a][0], kpts[chk.a][1]),
                    (kpts[chk.b][0], kpts[chk.b][1]),
                )
                tgt = chk.target_deg if chk.target_deg is not None else 0.0
                tol = chk.tol_deg if chk.tol_deg is not None else DEFAULT_ANGLE_TOL_DEG
                ok = abs(val - tgt) <= tol

                results.append(ok)
                parts.append(f"{chk.name}={val:.1f} tgt={tgt:.1f} tol={tol:.1f} ok={ok}")

            elif chk.type == "y_above":
                if not (chk.a and chk.b):
                    results.append(False); parts.append(f"{chk.name}=BAD_ARGS"); continue
                if chk.a not in kpts or chk.b not in kpts:
                    results.append(False); parts.append(f"{chk.name}=MISSING"); continue

                margin = chk.margin_norm if chk.margin_norm is not None else 0.0
                ya = kpts[chk.a][1]
                yb = kpts[chk.b][1]
                ok = (ya + margin) < yb
                results.append(ok)
                parts.append(f"{chk.name} ok={ok} (ya={ya:.3f} yb={yb:.3f} margin={margin:.3f})")

            else:
                results.append(False)
                parts.append(f"{chk.name}=UNKNOWN_TYPE({chk.type})")

        if not results:
            return False, "(no_checks)"

        final_ok = any(results) if self._cfg.logic == "any" else all(results)
        return final_ok, "(" + " | ".join(parts) + f" | logic={self._cfg.logic})"


def main():
    rclpy.init()
    node = OpenCVEvalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
