#include <chrono>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "server_web_interfaces/srv/ui_command.hpp"

#include "nao_led_interfaces/action/leds_play.hpp"
#include "nao_ui_utils/yaml_profile_loader.hpp"
#include "nao_ui_utils/led_profile.hpp"

using namespace std::chrono_literals;

namespace {
using UiCommandSrv = server_web_interfaces::srv::UiCommand;

using LedsPlay = nao_led_interfaces::action::LedsPlay;
using GoalHandleLedsPlay = rclcpp_action::ClientGoalHandle<LedsPlay>;

static std::string to_lower(std::string s) {
  for (auto & c : s) c = static_cast<char>(::tolower(c));
  return s;
}

static std::optional<std::string> json_get_string(const std::string & json, const std::string & key) {
  /*
  A very naive JSON string extractor for a flat JSON object. It looks for the pattern "key": "value" 
  and extracts value. It does not handle escaping, nested objects, arrays, or any JSON complexities. 
  It is only suitable for simple cases where we control the input format. For example, if json is 
  '{"state": "speak", "hold": 2.0}', then json_get_string(json, "state") would return "speak".
  */
  const std::string k = "\"" + key + "\"";
  auto p = json.find(k);
  if (p == std::string::npos) return std::nullopt;
  p = json.find(':', p);
  if (p == std::string::npos) return std::nullopt;
  p = json.find('"', p);
  if (p == std::string::npos) return std::nullopt;
  auto q = json.find('"', p + 1);
  if (q == std::string::npos) return std::nullopt;
  return json.substr(p + 1, q - (p + 1));
}

static double json_get_number(const std::string & json, const std::string & key, double def) {
  /*
  A very naive JSON number extractor for a flat JSON object. It looks for the pattern "key": value
  and extracts value as a number. It does not handle nested objects, arrays, or any JSON complexities.
  It is only suitable for simple cases where we control the input format. For example, if json is 
  '{"state": "speak", "hold": 2.0}', then json_get_number(json, "hold", 0.0) would return 2.0.
  */
  const std::string k = "\"" + key + "\"";
  auto p = json.find(k);
  if (p == std::string::npos) return def;
  p = json.find(':', p);
  if (p == std::string::npos) return def;
  auto b = json.find_first_of("0123456789.-", p);
  if (b == std::string::npos) return def;
  auto e = json.find_first_not_of("0123456789.-", b);
  try { return std::stod(json.substr(b, e - b)); } catch (...) { return def; }
}

LedsPlay::Goal to_goal(const nao_ui_utils::LedProfile & p) {
  LedsPlay::Goal g;
  g.leds = p.leds;
  g.mode = p.mode;
  g.frequency = p.frequency;
  g.duration = p.duration;
  g.colors = p.colors;
  g.intensities = p.intensities;
  return g;
}

static rclcpp::Time now_time(rclcpp::Node * n) { return n->get_clock()->now(); }

}  // namespace

class UiStub : public rclcpp::Node
{
public:
  UiStub() : rclcpp::Node("ui_stub")
  {
    pub_result_ = create_publisher<std_msgs::msg::String>("/hri/ui_result", 10);

    ui_client_ = create_client<UiCommandSrv>("/robot/ui/command");
    led_client_ = rclcpp_action::create_client<LedsPlay>(this, "leds_play");

    sub_ = create_subscription<std_msgs::msg::String>(
      "/hri/ui_cmd", 10,
      [this](const std_msgs::msg::String & msg) {
        // Enqueue the command (JSON) for processing in the main loop, no blocking
        enqueue_cmd(msg.data);
      });

    declare_parameter<std::string>("profiles_pkg", "nao_led_profiles");
    declare_parameter<std::string>("profiles_rel_dir", "config/emotions");

    // timeouts / retries
    declare_parameter<double>("ui_wait_service_s", 0.5);
    declare_parameter<double>("ui_future_timeout_s", 2.0);

    declare_parameter<double>("led_wait_action_s", 0.2);
    declare_parameter<int>("led_retries", 2);

    tick_timer_ = create_wall_timer(100ms, [this]() { tick(); });

    RCLCPP_INFO(get_logger(), "ui_stub ready. Subscribed to /hri/ui_cmd.");
  }

private:
  enum class Phase {
    IDLE,
    WAIT_UI_SERVICE,
    WAIT_UI_FUTURE,
    LED_ATTEMPT,
    HOLD_RUNNING,
    DONE_SUCCESS,
    DONE_FAILURE
  };

  struct Job {
    std::string json;
    std::string state;
    double hold_s{0.0};
    int led_attempts_left{0};
    rclcpp::Time t_start;
    rclcpp::Time t_phase_deadline;
    rclcpp::Time t_hold_end;
    rclcpp::Client<UiCommandSrv>::SharedFuture ui_future;
    bool ui_future_valid{false};
  };

  void publish_result(const std::string & v) {
    std_msgs::msg::String out;
    out.data = v;
    pub_result_->publish(out);
  }

  std::string map_led_profile(const std::string & state) const {
    if (state == "speak") return "speak";
    if (state == "video") return "video";
    if (state == "comparison") return "comparison";
    if (state == "nao_pos") return "nao_pos";
    return "neutral";
  }

  void enqueue_cmd(const std::string & raw_json) {
    // If job in progress, enqueue for later, else start immediately
    pending_.push_back(raw_json);
    if (phase_ == Phase::IDLE) {
      start_next_job();
    }
  }

  void start_next_job() {
    // If no pending jobs, go idle
    if (pending_.empty()) {
      phase_ = Phase::IDLE;
      return;
    }

    Job j;
    j.json = pending_.front();
    pending_.pop_front();

    j.state = to_lower(json_get_string(j.json, "state").value_or("default"));
    // If "hold" or "duration" field is present, we consider it a hold with the specified duration, else no hold.
    if (j.json.find("\"hold\"") != std::string::npos) {
      j.hold_s = json_get_number(j.json, "hold", 0.0);
    } else if (j.json.find("\"duration\"") != std::string::npos) {
      j.hold_s = json_get_number(j.json, "duration", 0.0);
    } else {
      j.hold_s = 0.0;
    }
    // Try to get retries from JSON, if not present use parameter, if parameter not present use default 2
    j.led_attempts_left = get_parameter("led_retries").as_int();

    j.t_start = now_time(this);
    job_ = j;

    phase_ = Phase::WAIT_UI_SERVICE;
    job_.t_phase_deadline = job_.t_start + rclcpp::Duration::from_seconds(get_parameter("ui_wait_service_s").as_double());

    publish_result("running");
  }

  void fail_job_and_continue() {
    publish_result("failure");
    // Consume the UI future if it was valid, to avoid blocking the executor with an unconsumed future
    job_.ui_future_valid = false;
    phase_ = Phase::DONE_FAILURE;
  }

  void succeed_job_and_continue() {
    publish_result("success");
    job_.ui_future_valid = false;
    phase_ = Phase::DONE_SUCCESS;
  }

  void tick() {
    /*
    Main loop, state machine for processing the current job and its phases, with timeouts and retries.
      - IDLE: no job, waiting for commands
      - WAIT_UI_SERVICE: waiting for the UI service to be available, with timeout. If available,
        send request and go to WAIT_UI_FUTURE
      - WAIT_UI_FUTURE: waiting for the UI service response, with timeout. If response received and success=true, 
          go to LED_ATTEMPT, else fail. If timeout, fail.
      - LED_ATTEMPT: try to send LED goal, if fails retry a configurable number of times, then continue anyway
         (LED is best-effort). If hold time specified, go to HOLD_RUNNING, else succeed immediately.
      - HOLD_RUNNING: just wait until hold time is over, then succeed.
      - DONE_SUCCESS / DONE_FAILURE: terminal phases, wait a tick and then start the next job if any.
    */
    switch (phase_) {
      case Phase::IDLE:
        return;

      case Phase::DONE_SUCCESS:
      case Phase::DONE_FAILURE:
        start_next_job();
        return;

      case Phase::WAIT_UI_SERVICE: {
        publish_result("running");

        if (ui_client_->service_is_ready()) {
          auto req = std::make_shared<UiCommandSrv::Request>();
          req->json = job_.json;

          // async_send_request devuelve FutureAndRequestId (deprecated si se convierte implícitamente)
          auto far = ui_client_->async_send_request(req);
          job_.ui_future = far.future.share();
          job_.ui_future_valid = true;

          phase_ = Phase::WAIT_UI_FUTURE;
          job_.t_phase_deadline =
            now_time(this) + rclcpp::Duration::from_seconds(get_parameter("ui_future_timeout_s").as_double());
          return;
        }


        if (now_time(this) > job_.t_phase_deadline) {
          RCLCPP_ERROR(get_logger(), "UI service /robot/ui/command not available (timeout). Continuing.");
          fail_job_and_continue();
          return;
        }
        return;
      }

      /*
      -Case WAIT_UI_FUTURE: we wait for the future to be ready. If it becomes ready, we check the response.
        If success=true, we proceed to LED_ATTEMPT. If success=false, we log the error and fail the job. 
        If the future is not ready and we exceed the timeout, we log the timeout and fail the job. 
        We also publish "running" status while waiting.
      -Case LED_ATTEMPT: we try to send the LED goal. If it fails, we decrement the attempts counter and if 
      it reaches zero, we log a warning and continue (LED is best-effort). If it succeeds, we check if we 
      need to hold. If hold time is specified, we go to HOLD_RUNNING phase, else we succeed immediately.
      -Case HOLD_RUNNING: we just wait until the current time exceeds the hold end time, then we succeed.
      -Cases DONE_SUCCESS and DONE_FAILURE: these are terminal phases where we have finished processing a job.
        We call start_next_job() to check if there are more jobs in the queue and start the next one if available.
      -Case IDLE: we do nothing and just wait for commands.
      */
      case Phase::WAIT_UI_FUTURE: {
        publish_result("running");

        if (!job_.ui_future_valid) {
          fail_job_and_continue();
          return;
        }

        if (job_.ui_future.wait_for(0s) == std::future_status::ready) {
          auto resp = job_.ui_future.get();
          if (!resp->success) {
            RCLCPP_ERROR(get_logger(), "UI service response: success=false msg='%s'", resp->message.c_str());
            fail_job_and_continue();
            return;
          }
          phase_ = Phase::LED_ATTEMPT;
          return;
        }

        if (now_time(this) > job_.t_phase_deadline) {
          RCLCPP_ERROR(get_logger(), "UI service call timeout. Continuing.");
          fail_job_and_continue();
          return;
        }
        return;
      }

      case Phase::LED_ATTEMPT: {
        publish_result("running");

        const std::string led_profile = map_led_profile(job_.state);
        bool led_sent = send_led_goal_best_effort(led_profile);

        if (!led_sent) {
          job_.led_attempts_left--;
          if (job_.led_attempts_left <= 0) {
            RCLCPP_WARN(get_logger(), "LED not available after retries. Continuing.");
            if (job_.hold_s > 0.0) {
              phase_ = Phase::HOLD_RUNNING;
              job_.t_hold_end = now_time(this) + rclcpp::Duration::from_seconds(job_.hold_s);
            } else {
              succeed_job_and_continue();
            }
            return;
          }
          return;
        }

        if (job_.hold_s > 0.0) {
          phase_ = Phase::HOLD_RUNNING;
          job_.t_hold_end = now_time(this) + rclcpp::Duration::from_seconds(job_.hold_s);
        } else {
          succeed_job_and_continue();
        }
        return;
      }

      case Phase::HOLD_RUNNING: {
        publish_result("running");
        if (now_time(this) >= job_.t_hold_end) {
          succeed_job_and_continue();
        }
        return;
      }

      default:
        return;
    }
  }

  bool send_led_goal_best_effort(const std::string & profile_name)
  {
    // Wait for action server with timeout
    const double wait_action = get_parameter("led_wait_action_s").as_double();
    if (!led_client_->wait_for_action_server(std::chrono::duration<double>(wait_action))) {
      return false;
    }

    const std::string pkg = get_parameter("profiles_pkg").as_string();
    const std::string rel = get_parameter("profiles_rel_dir").as_string();

    std::string error;
    auto dir_opt = nao_ui_utils::resolve_in_share(pkg, rel, error);
    if (!dir_opt) {
      RCLCPP_WARN(get_logger(), "LED profiles not resolved: %s", error.c_str());
      return false;
    }

    const std::string yaml_path = *dir_opt + "/" + profile_name + ".yml";
    std::string load_error;
    auto profile_opt = nao_ui_utils::load_profile_from_yaml_file(yaml_path, load_error);
    if (!profile_opt) {
      RCLCPP_WARN(get_logger(), "LED profile load failed '%s': %s", yaml_path.c_str(), load_error.c_str());
      return false;
    }

    auto goal_msg = to_goal(*profile_opt);

    rclcpp_action::Client<LedsPlay>::SendGoalOptions opts;
    opts.goal_response_callback = [this](GoalHandleLedsPlay::SharedPtr gh) {
      if (!gh) RCLCPP_WARN(get_logger(), "LED goal rejected");
    };
    opts.result_callback = [](const GoalHandleLedsPlay::WrappedResult &) {};

    led_client_->async_send_goal(goal_msg, opts);
    return true;
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_result_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::Client<UiCommandSrv>::SharedPtr ui_client_;
  rclcpp_action::Client<LedsPlay>::SharedPtr led_client_;
  rclcpp::TimerBase::SharedPtr tick_timer_;

  /// Queue of pending commands (JSON strings) to process sequentially
  std::deque<std::string> pending_;
  Job job_;
  Phase phase_{Phase::IDLE};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UiStub>());
  rclcpp::shutdown();
  return 0;
}