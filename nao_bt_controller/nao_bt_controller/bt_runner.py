#!/usr/bin/env python3
import json
import os
import time
import yaml
import rclpy
import py_trees

from rclpy.node import Node
from std_msgs.msg import String


# -------------------- Payload helpers --------------------
def ui_payload(state: str, emotion: str = None, text: str = None, video: str = None, hold: float = None):
    d = {"state": state}
    if emotion is not None:
        d["emotion"] = emotion
    if text is not None:
        d["text"] = text
    if video is not None:
        d["video"] = video
    if hold is not None:
        d["hold"] = float(hold)
    return json.dumps(d, ensure_ascii=False)

# Speak payload is just the text to speak, but as a JSON for extensibility
def speak_payload(text: str) -> str:
    return json.dumps({"text": text}, ensure_ascii=False)


def nao_pos_payload(pos_file: str, duration_s: float) -> str:
    return json.dumps({"file": pos_file, "duration": float(duration_s)}, ensure_ascii=False)


# -------------------- LED profiles --------------------
EMOTION_LED_CFG = {
    "happy": "happy.yml",
    "neutral": "neutral.yml",
    "sad": "sad.yml",
    "serious": "serious.yml",
    "surprised": "surprised.yml",
}

# -----------------------State LED profile-----------------------
STATE_LED_CFG = {
    "default": "neutral.yml",
    "speak": "speak.yml",
    "video": "video.yml",
    "comparison": "comparison.yml",
    "nao_pos": "nao_pos.yml",
}


# -------------------- ROS IO --------------------
class RosIo(Node):
    def __init__(self):
        super().__init__("nao_hri_bt_runner")

        self.pub_speak = self.create_publisher(String, "/hri/speak_cmd", 10)
        self.pub_pos = self.create_publisher(String, "/hri/nao_pos_cmd", 10)
        self.pub_eval = self.create_publisher(String, "/hri/eval_cmd", 10)
        self.pub_ui = self.create_publisher(String, "/hri/ui_cmd", 10)
        self.pub_leds = self.create_publisher(String, "/hri/leds_cmd", 10)

        self.last_ui = None
        self.last_speak = None
        self.last_pos = None
        self.last_eval = None

        self.create_subscription(String, "/hri/ui_result", self._on_ui, 10)
        self.create_subscription(String, "/hri/speak_result", self._on_speak, 10)
        self.create_subscription(String, "/hri/nao_pos_result", self._on_pos, 10)
        self.create_subscription(String, "/hri/eval_result", self._on_eval, 10)

    def _on_ui(self, msg: String):
        self.last_ui = (msg.data or "").strip().lower()

    def _on_speak(self, msg: String):
        self.last_speak = (msg.data or "").strip().lower()

    def _on_pos(self, msg: String):
        self.last_pos = (msg.data or "").strip().lower()

    def _on_eval(self, msg: String):
        self.last_eval = (msg.data or "").strip().lower()

    def publish(self, pub, payload: str):
        m = String()
        m.data = payload
        pub.publish(m)

    def publish_leds_emotion(self, emotion: str):
        e = (emotion or "").strip().lower()
        if not e:
            return
        cfg = EMOTION_LED_CFG.get(e, f"{e}.yml")
        self.publish(self.pub_leds, cfg)

    def publish_leds_state(self, state: str):
        s = (state or "").strip().lower()
        if not s:
            return
        cfg = STATE_LED_CFG.get(s, f"{s}.yml")
        self.publish(self.pub_leds, cfg)


# -------------------- Generic behaviours --------------------
class TopicCommandWait(py_trees.behaviour.Behaviour):
    """
    Publish 1 command and wait for feedback:
      - success -> SUCCESS
      - failure -> FAILURE
      - running/None -> RUNNING until timeout 
    """
    def __init__(self, name: str, ros: RosIo, kind: str, payload: str,
                 timeout_s: float = 30.0, soft_fail: bool = False):
        super().__init__(name)
        self.ros = ros
        self.kind = kind
        self.payload = payload
        self.timeout_s = float(timeout_s)
        self.soft_fail = bool(soft_fail)
        self._t0 = None
        self._sent = False

    def initialise(self):
        self._t0 = time.time()
        self._sent = False

        if self.kind == "ui":
            self.ros.last_ui = None
        elif self.kind == "speak":
            self.ros.last_speak = None
        elif self.kind == "pos":
            self.ros.last_pos = None
        elif self.kind == "eval":
            self.ros.last_eval = None

    def _get_last(self):
        if self.kind == "ui":
            return self.ros.last_ui
        if self.kind == "speak":
            return self.ros.last_speak
        if self.kind == "pos":
            return self.ros.last_pos
        if self.kind == "eval":
            return self.ros.last_eval
        return None

    def _publish(self):
        if self.kind == "ui":
            self.ros.publish(self.ros.pub_ui, self.payload)
        elif self.kind == "speak":
            self.ros.publish(self.ros.pub_speak, self.payload)
        elif self.kind == "pos":
            self.ros.publish(self.ros.pub_pos, self.payload)
        elif self.kind == "eval":
            self.ros.publish(self.ros.pub_eval, self.payload)

    def update(self):
        if not self._sent:
            self._publish()
            self._sent = True
            return py_trees.common.Status.RUNNING

        last = self._get_last()

        if last == "success":
            return py_trees.common.Status.SUCCESS

        if last == "failure":
            return py_trees.common.Status.SUCCESS if self.soft_fail else py_trees.common.Status.FAILURE

        if last == "running" or last is None or last == "":
            if (time.time() - self._t0) > self.timeout_s:
                return py_trees.common.Status.SUCCESS if self.soft_fail else py_trees.common.Status.FAILURE
            return py_trees.common.Status.RUNNING

        if (time.time() - self._t0) > self.timeout_s:
            return py_trees.common.Status.SUCCESS if self.soft_fail else py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

# Simple timer used for attempt windows and cooldowns, where no feedback is expected, just wait the duration.
class SimpleTimer(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, duration: float):
        super().__init__(name)
        self.duration = float(duration)
        self._t0 = None

    def initialise(self):
        self._t0 = time.time()

    def update(self):
        if self._t0 is None:
            self._t0 = time.time()
        if (time.time() - self._t0) >= self.duration:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


# -------------------- Speak hard-gate (retry on failure) --------------------
class SpeakUntilSuccess(py_trees.behaviour.Behaviour):
    """
    Publish speak command and wait for 'success' feedback to return SUCCESS.
    """
    def __init__(self, name: str, ros: RosIo, payload: str,
                 attempt_timeout_s: float = 30.0, retry_backoff_s: float = 0.6):
        super().__init__(name)
        self.ros = ros
        self.payload = payload
        self.attempt_timeout_s = float(attempt_timeout_s)
        self.retry_backoff_s = float(retry_backoff_s)

        self._sent = False
        self._t0 = None
        self._next_retry_t = 0.0

    def initialise(self):
        self.ros.last_speak = None
        self._sent = False
        self._t0 = time.time()
        self._next_retry_t = 0.0

    def update(self):
        now = time.time()
        last = self.ros.last_speak

        # If success, return SUCCESS immediately
        if last == "success":
            return py_trees.common.Status.SUCCESS

        # If failure or timeout, reset and retry (with backoff) without returning FAILURE yet
        timed_out = (now - self._t0) > self.attempt_timeout_s
        if last == "failure" or timed_out:
            self.ros.get_logger().warn(
                f"[SpeakUntilSuccess] attempt failed (last={last}, timeout={timed_out}). Retrying..."
            )
            self.ros.last_speak = None
            self._sent = False
            self._t0 = now
            self._next_retry_t = now + self.retry_backoff_s
            return py_trees.common.Status.RUNNING

        # If not sent yet, send the command (but only if we're past the backoff time)
        if not self._sent:
            if now < self._next_retry_t:
                return py_trees.common.Status.RUNNING
            self.ros.publish(self.ros.pub_speak, self.payload)
            self._sent = True
            return py_trees.common.Status.RUNNING

        return py_trees.common.Status.RUNNING


# -------------------- Eval gate --------------------
class EvalGate(py_trees.behaviour.Behaviour):
    """
    Eval return "true"/"false".
    - SEns JSON {"id": pose_id}.
    - Timeout -> FAILURE.
    """
    def __init__(self, name: str, ros: RosIo, pose_id: str, timeout_s: float = 10.0):
        super().__init__(name)
        self.ros = ros
        self.pose_id = str(pose_id)
        self.timeout_s = float(timeout_s)
        self._t0 = None
        self._sent = False

    def initialise(self):
        self._t0 = time.time()
        self._sent = False
        self.ros.last_eval = None

    def update(self):
        if not self._sent:
            payload = json.dumps({"id": self.pose_id}, ensure_ascii=False)
            self.ros.publish(self.ros.pub_eval, payload)
            self._sent = True
            return py_trees.common.Status.RUNNING

        last = self.ros.last_eval
        if last == "true":
            return py_trees.common.Status.SUCCESS
        if last == "false":
            return py_trees.common.Status.FAILURE

        if (time.time() - self._t0) > self.timeout_s:
            self.ros.last_eval = "false"
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING


class RetryUntilSuccess(py_trees.decorators.Decorator):
    def __init__(self, name: str, child: py_trees.behaviour.Behaviour, max_failures: int = 0):
        super().__init__(name=name, child=child)
        self.max_failures = int(max_failures)  # 0 => infinite retries
        self._failures = 0

    def update(self):
        status = self.decorated.status

        if status == py_trees.common.Status.SUCCESS:
            return py_trees.common.Status.SUCCESS

        if status == py_trees.common.Status.FAILURE:
            self._failures += 1
            if self.max_failures > 0 and self._failures >= self.max_failures:
                return py_trees.common.Status.FAILURE
            self.decorated.stop(py_trees.common.Status.INVALID)
            return py_trees.common.Status.RUNNING

        return py_trees.common.Status.RUNNING


# -------------------- LED/UI helpers --------------------
class SetEmotionUiOnly(py_trees.behaviour.Behaviour):
    """
    Only set UI emotion (no LED change) for a 
    soft visual feedback of the current emotion/mode.
    """
    def __init__(self, name: str, ros: RosIo, emotion: str):
        super().__init__(name)
        self.ros = ros
        self.emotion = (emotion or "").strip().lower()
        self._done = False

    def initialise(self):
        self._done = False

    def update(self):
        if not self._done:
            if self.emotion:
                self.ros.publish(self.ros.pub_ui, ui_payload("default", emotion=self.emotion))
            self._done = True
        return py_trees.common.Status.SUCCESS


class SetStateLeds(py_trees.behaviour.Behaviour):
    """
    Force the LED profile by ESTADO/MODO (no emoción) for a strong visual feedback of the current mode.
    """
    def __init__(self, name: str, ros: RosIo, state: str):
        super().__init__(name)
        self.ros = ros
        self.state = (state or "").strip().lower()
        self._done = False

    def initialise(self):
        self._done = False

    def update(self):
        if not self._done:
            if self.state:
                self.ros.publish_leds_state(self.state)
            self._done = True
        return py_trees.common.Status.SUCCESS


class SetEmotionLeds(py_trees.behaviour.Behaviour):
    """ 
    Force the LED profile by EMOCIÓN (no estado) for a strong visual feedback of the current emotion.
    """
    def __init__(self, name: str, ros: RosIo, emotion: str):
        super().__init__(name)
        self.ros = ros
        self.emotion = (emotion or "").strip().lower()
        self._done = False

    def initialise(self):
        self._done = False

    def update(self):
        if not self._done:
            if self.emotion:
                self.ros.publish_leds_emotion(self.emotion)
            self._done = True
        return py_trees.common.Status.SUCCESS


UI_CLEAR = ui_payload("default", emotion="neutral")


# -------------------- Composite blocks --------------------
def speak_block(ros: RosIo, name_prefix: str, text: str, ui_emotion: str = "neutral"):
    """
      Leds for "speak" state + UI emotion for visual feedback during speaking + 
      speak command with hard gate + clear UI
    """
    seq = py_trees.composites.Sequence(name=f"{name_prefix}_speak_block", memory=True)

    seq.add_children([
        SetStateLeds(f"leds_state_speak_{name_prefix}", ros, "speak"),

        # Set UI emotion for visual feedback during speaking (soft, can fail without breaking the flow)
        SetEmotionUiOnly(f"ui_emotion_only_{name_prefix}_{ui_emotion}", ros, ui_emotion),

        # UI "speak" (soft)
        TopicCommandWait(
            f"ui_speak_{name_prefix}",
            ros,
            "ui",
            ui_payload("speak", emotion=ui_emotion, text=text),
            timeout_s=4.0,
            soft_fail=True,
        ),

        # Speak HARD gate (retry on failure)
        SpeakUntilSuccess(
            f"speak_until_success_{name_prefix}",
            ros,
            speak_payload(text),
            attempt_timeout_s=30.0,
            retry_backoff_s=0.7,
        ),

        # Clear UI after speaking (soft)
        TopicCommandWait(
            f"ui_clear_after_{name_prefix}",
            ros,
            "ui",
            UI_CLEAR,
            timeout_s=2.0,
            soft_fail=True,
        ),
    ])

    return seq


def nao_pos_with_relax(
        
    ros: RosIo,
    name_prefix: str,
    demo_pos_file: str,
    demo_duration_s: float,
    relax_pos_file: str,
    relax_duration_s: float,
    timeout_s: float = 20.0,
):
    """
    Execute a NAO pose demo command, followed by a relax pose, to show the user the target pose and then 
    relax to a neutral position before the user's attempt.
    Both commands have a hard gate with timeout, but if they fail,
    we just continue with the flow (soft_fail=True)
    """
    seq = py_trees.composites.Sequence(name=f"{name_prefix}_nao_pos_with_relax", memory=True)
    seq.add_children([
        SetStateLeds(f"leds_state_nao_pos_{name_prefix}", ros, "nao_pos"),
        TopicCommandWait(
            f"{name_prefix}_nao_pos_demo",
            ros,
            "pos",
            nao_pos_payload(demo_pos_file, demo_duration_s),
            timeout_s=timeout_s,
            soft_fail=True,
        ),
        TopicCommandWait(
            f"{name_prefix}_nao_pos_relax",
            ros,
            "pos",
            nao_pos_payload(relax_pos_file, relax_duration_s),
            timeout_s=timeout_s,
            soft_fail=True,
        ),
        SetStateLeds(f"leds_state_default_after_nao_pos_{name_prefix}", ros, "default"),
    ])
    return seq


# -------------------- Catalog --------------------
def load_exercises_catalog(path: str) -> dict:
    """ 
    Load the exercises catalog from a YAML file. The catalog should be a dict of dicts, 
    where each key is a pose_id and each value is a dict with the following
    """
    if not path or not os.path.isfile(path):
        raise FileNotFoundError(f"exercises_catalog.yml no encontrado: {path}")
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict) or not data:
        raise ValueError(f"exercises_catalog.yml vacío o inválido: {path}")

    out = {}
    for k, v in data.items():
        if not isinstance(k, str) or not k.strip():
            continue
        if not isinstance(v, dict):
            continue
        out[k.strip()] = {
            "video": str(v.get("video", "")).strip(),
            "eval_template": str(v.get("eval_template", "")).strip(),
            "nao_pos_cmd": str(v.get("nao_pos_cmd", "")).strip(),
            "nao_pos_file": str(v.get("nao_pos_file", "")).strip(),
        }
    if not out:
        raise ValueError(f"exercises_catalog.yml sin entradas válidas: {path}")
    return out


# -------------------- Tree builders --------------------
def make_intro(ros: RosIo):
    seq = py_trees.composites.Sequence(name="intro", memory=True)
    seq.add_children([
        speak_block(
            ros,
            "intro",
            "Hoy vamos a hacer este ejercicio, se hace así:",
            ui_emotion="neutral",
        ),
    ])
    return seq


def make_attempt_cycle(
    ros: RosIo,
    pose_id: str,
    catalog: dict,
    attempt_window_s: float,
    eval_timeout_s: float,
    demo_pos_duration_s: float,
    relax_pos_file: str,
    relax_pos_duration_s: float,
):
    """
    Build the attempt cycle, which consists of:
    1) Intro + demo (no eval, just show the user the target pose with a demo and a video,
        with visual feedback through LEDs and UI)
    2) Attempt window (no eval yet, just show the user a "comparison" video and wait for their attempt,
        with visual feedback through LEDs and UI)
    3) Eval (once)
    4) Resultado (good/bad) with different branches for good and bad results, and visual feedback through 
        LEDs and UI in each case.
    """
    attempt = py_trees.composites.Sequence(name="attempt", memory=True)

    if pose_id not in catalog:
        pose_id = next(iter(catalog.keys()))

    pose_info = catalog[pose_id]
    video_file = pose_info.get("video", "") or ""
    demo_pos_file = pose_info.get("nao_pos_file", "") or f"{pose_id}.pos"

    attempt.add_children([
        # Default LEDs por estado "default" (no emoción) for the intro and demo, to differentiate from the attempt and eval phases
        SetStateLeds("leds_state_default_start", ros, "default"),
        TopicCommandWait(
            "ui_pose_neutral",
            ros,
            "ui",
            ui_payload("default", emotion="neutral"),
            timeout_s=2.0,
            soft_fail=True,
        ),

        # Pose demo + relax
        nao_pos_with_relax(
            ros,
            name_prefix="demo",
            demo_pos_file=demo_pos_file,
            demo_duration_s=float(demo_pos_duration_s),
            relax_pos_file=relax_pos_file,
            relax_duration_s=float(relax_pos_duration_s),
            timeout_s=20.0,
        ),

        speak_block(
            ros,
            "pre_video",
            "Ahora te voy a enseñar un video.",
            ui_emotion="neutral",
        ),

        # 2) VIDEO: LEDs for video state + UI video with the exercise video
        SetStateLeds("leds_state_video", ros, "video"),
        TopicCommandWait(
            "ui_video",
            ros,
            "ui",
            ui_payload("video", emotion="neutral", video=video_file, hold=8.0),
            timeout_s=12.0,
            soft_fail=True,
        ),

        speak_block(
            ros,
            "your_turn",
            "Ahora te toca a ti.",
            ui_emotion="neutral",
        ),
    ])

    # Set LEDs for attempt/comparison state during the attempt window,
    wait_attempt = py_trees.composites.Parallel(
        name="wait_attempt_window",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=True),
    )
    wait_attempt.add_children([
        # COMPARISON: LEDs for comparison state + UI with the comparison video
        TopicCommandWait(
            "ui_comparison",
            ros,
            "ui",
            ui_payload("comparison", emotion="neutral", video=video_file, hold=float(attempt_window_s)),
            timeout_s=2.0,
            soft_fail=True,
        ),
        SimpleTimer(name="attempt_timer", duration=float(attempt_window_s)),
    ])

    # Eval, with hard gate and timeout, to not wait indefinitely for the user's attempt 
    eval_gate = EvalGate("eval_gate", ros, pose_id=pose_id, timeout_s=float(eval_timeout_s))

    # Result branch: if good -> good_path, else -> bad_path
    decision = py_trees.composites.Selector(name="decision", memory=True)

    good_path = py_trees.composites.Sequence(name="good_path", memory=True)
    good_path.add_children([
        speak_block(
            ros,
            "good",
            "Muy bien, lo has hecho correctamente.",
            ui_emotion="happy",
        ),

        # HAPPY final
        SetEmotionLeds("leds_emotion_happy_final", ros, "happy"),
        TopicCommandWait(
            "ui_hold_happy",
            ros,
            "ui",
            ui_payload("default", emotion="happy"),
            timeout_s=3.0,
            soft_fail=True,
        ),

        # Back to neutral
        SetEmotionLeds("leds_emotion_neutral_after_good", ros, "neutral"),
        TopicCommandWait(
            "ui_clear_after_good",
            ros,
            "ui",
            ui_payload("default", emotion="neutral"),
            timeout_s=2.0,
            soft_fail=True,
        ),

        py_trees.behaviours.Success(name="end_success"),
    ])

    bad_path = py_trees.composites.Sequence(name="bad_path", memory=True)
    bad_path.add_children([
        speak_block(
            ros,
            "bad",
            "Así no es, es de la siguiente manera.",
            ui_emotion="neutral",
        ),

        nao_pos_with_relax(
            ros,
            name_prefix="fail_relax",
            demo_pos_file=relax_pos_file,
            demo_duration_s=float(relax_pos_duration_s),
            relax_pos_file=relax_pos_file,
            relax_duration_s=0.2,
            timeout_s=10.0,
        ),

        SimpleTimer(name="cooldown_after_fail", duration=2.0),
        py_trees.behaviours.Failure(name="end_failure"),
    ])

    success_branch = py_trees.composites.Sequence(name="if_good", memory=True)
    success_branch.add_children([wait_attempt, eval_gate, good_path])

    decision.add_children([success_branch, bad_path])
    attempt.add_children([decision])
    return attempt


def build_tree(
    ros: RosIo,
    catalog: dict,
    pose_id: str,
    attempt_window_s: float,
    eval_timeout_s: float,
    demo_pos_duration_s: float,
    relax_pos_file: str,
    relax_pos_duration_s: float,
) -> py_trees.behaviour.Behaviour:
    root = py_trees.composites.Sequence(name="root", memory=True)
    root.add_children([
        make_intro(ros),
        RetryUntilSuccess(
            name="repeat_until_good",
            child=make_attempt_cycle(
                ros,
                pose_id=pose_id,
                catalog=catalog,
                attempt_window_s=attempt_window_s,
                eval_timeout_s=eval_timeout_s,
                demo_pos_duration_s=demo_pos_duration_s,
                relax_pos_file=relax_pos_file,
                relax_pos_duration_s=relax_pos_duration_s,
            ),
            max_failures=0,
        )
    ])
    return root


# -------------------- main --------------------
def main():
    rclpy.init()
    ros = RosIo()

    ros.declare_parameter(
        "catalog_path",
        "/home/rubenb/uni/5_ano/1cuatri/tfg/ubunt22/nao_ws/src/thirdparty/nao_hri/nao_hri_demo_nodes/nao_hri_demo_nodes/config/exercises_catalog.yml",
    )
    ros.declare_parameter("pose_id", "rightarmUp")
    ros.declare_parameter("attempt_window_s", 6.0)
    ros.declare_parameter("eval_timeout_s", 12.0)
    ros.declare_parameter("demo_pos_duration_s", 2.0)
    ros.declare_parameter("relax_pos_file", "armsDown.pos")
    ros.declare_parameter("relax_pos_duration_s", 1.5)

    catalog_path = ros.get_parameter("catalog_path").get_parameter_value().string_value
    pose_id = ros.get_parameter("pose_id").get_parameter_value().string_value
    attempt_window_s = float(ros.get_parameter("attempt_window_s").value)
    eval_timeout_s = float(ros.get_parameter("eval_timeout_s").value)
    demo_pos_duration_s = float(ros.get_parameter("demo_pos_duration_s").value)
    relax_pos_file = ros.get_parameter("relax_pos_file").get_parameter_value().string_value
    relax_pos_duration_s = float(ros.get_parameter("relax_pos_duration_s").value)

    catalog = load_exercises_catalog(catalog_path)

    ros.get_logger().info(f"[BT] catalog_path={catalog_path}")
    ros.get_logger().info(f"[BT] pose_id={pose_id}")
    ros.get_logger().info(f"[BT] attempt_window_s={attempt_window_s}")
    ros.get_logger().info(f"[BT] eval_timeout_s={eval_timeout_s}")
    ros.get_logger().info(f"[BT] demo_pos_duration_s={demo_pos_duration_s}")
    ros.get_logger().info(f"[BT] relax_pos_file={relax_pos_file}")
    ros.get_logger().info(f"[BT] relax_pos_duration_s={relax_pos_duration_s}")

    tree = py_trees.trees.BehaviourTree(
        build_tree(
            ros,
            catalog=catalog,
            pose_id=pose_id,
            attempt_window_s=attempt_window_s,
            eval_timeout_s=eval_timeout_s,
            demo_pos_duration_s=demo_pos_duration_s,
            relax_pos_file=relax_pos_file,
            relax_pos_duration_s=relax_pos_duration_s,
        )
    )

    tick_hz = 10.0
    period = 1.0 / tick_hz

    try:
        while rclpy.ok():
            rclpy.spin_once(ros, timeout_sec=0.0)
            tree.tick()
            if tree.root.status == py_trees.common.Status.SUCCESS:
                break
            time.sleep(period)
    except KeyboardInterrupt:
        pass

    ros.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
