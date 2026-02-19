#include <chrono>
#include <deque>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "server_web_interfaces/srv/ui_command.hpp"

#include "nao_led_interfaces/action/leds_play.hpp"
#include "nao_led_interfaces/msg/led_indexes.hpp"
#include "nao_led_interfaces/msg/led_modes.hpp"

#include "nao_ui_utils/led_profile.hpp"
#include "nao_ui_utils/yaml_profile_loader.hpp"

using namespace std::chrono_literals;

namespace {

using UiCommandSrv = server_web_interfaces::srv::UiCommand;

using LedsPlay = nao_led_interfaces::action::LedsPlay;
using GoalHandleLedsPlay = rclcpp_action::ClientGoalHandle<LedsPlay>;
using LedIndexes = nao_led_interfaces::msg::LedIndexes;
using LedModes = nao_led_interfaces::msg::LedModes;

static std::string to_lower(std::string s)
{
  for (auto & c : s) {
    c = static_cast<char>(::tolower(c));
  }
  return s;
}

// Very naive JSON string extractor for a flat JSON object.
// Looks for: "key": "value"
static std::optional<std::string> json_get_string(const std::string & json, const std::string & key)
{
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

// Very naive JSON number extractor for a flat JSON object.
// Looks for: "key": value
static double json_get_number(const std::string & json, const std::string & key, double def)
{
  const std::string k = "\"" + key + "\"";
  auto p = json.find(k);
  if (p == std::string::npos) return def;
  p = json.find(':', p);
  if (p == std::string::npos) return def;
  auto b = json.find_first_of("0123456789.-", p);
  if (b == std::string::npos) return def;
  auto e = json.find_first_not_of("0123456789.-", b);
  try {
    return std::stod(json.substr(b, e - b));
  } catch (...) {
    return def;
  }
}

static LedsPlay::Goal to_goal(const nao_ui_utils::LedProfile & p)
{
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

static bool ends_with(const std::string & s, const std::string & suf)
{
  if (s.size() < suf.size()) return false;
  return s.compare(s.size() - suf.size(), suf.size(), suf) == 0;
}

static inline bool eff_in_targets(uint8_t eff, const std::array<uint8_t, 2> & targets)
{
  return (targets[0] == eff) || (targets[1] == eff);
}

static inline std::array<std_msgs::msg::ColorRGBA, 8> all_off_colors()
{
  std::array<std_msgs::msg::ColorRGBA, 8> c{};
  for (auto & x : c) {
    x.r = 0.0F; x.g = 0.0F; x.b = 0.0F; x.a = 1.0F;
  }
  return c;
}

static inline std::array<float, 12> all_zero_intensities()
{
  std::array<float, 12> v{};
  for (auto & x : v) x = 0.0F;
  return v;
}

// Build a "turn off this effector" goal without touching server interfaces.
// Uses leds=[eff, eff], mode=STEADY, duration=0 so it applies and returns immediately.
static LedsPlay::Goal make_off_goal(uint8_t eff)
{
  LedsPlay::Goal g;
  g.leds = {eff, eff};
  g.mode = LedModes::STEADY;
  g.frequency = 0.0F;
  g.duration = 0.0F;

  g.colors = all_off_colors();
  g.intensities = all_zero_intensities();
  return g;
}

}  // namespace

class UiStub : public rclcpp::Node
{
public:
  UiStub() : rclcpp::Node("ui_stub")
  {
    pub_ui_result_ = create_publisher<std_msgs::msg::String>("/hri/ui_result", 10);

    ui_client_ = create_client<UiCommandSrv>("/robot/ui/command");
    led_client_ = rclcpp_action::create_client<LedsPlay>(this, "leds_play");

    sub_ui_cmd_ = create_subscription<std_msgs::msg::String>(
      "/hri/ui_cmd", 10,
      [this](const std_msgs::msg::String & msg) { enqueue_ui_cmd(msg.data); });

    sub_leds_cmd_ = create_subscription<std_msgs::msg::String>(
      "/hri/leds_cmd", 10,
      [this](const std_msgs::msg::String & msg) { on_leds_cmd(msg.data); });

    declare_parameter<std::string>("profiles_pkg", "nao_led_profiles");
    declare_parameter<std::string>("profiles_rel_dir", "config/emotions");

    declare_parameter<double>("ui_wait_service_s", 0.5);
    declare_parameter<double>("ui_future_timeout_s", 2.0);

    declare_parameter<double>("led_wait_action_s", 0.2);

    declare_parameter<bool>("preempt_ui_on_new_cmd", true);

    // NEW: when a profile arrives, turn OFF every effector not mentioned in it
    declare_parameter<bool>("led_scene_reset_others", true);

    tick_timer_ = create_wall_timer(50ms, [this]() { tick(); });

    RCLCPP_INFO(get_logger(), "ui_stub ready. /hri/ui_cmd->UI service, /hri/leds_cmd->leds_play");
  }

private:
  enum class Phase {
    IDLE,
    WAIT_UI_SERVICE,
    WAIT_UI_FUTURE,
    HOLD_RUNNING,
    DONE_SUCCESS,
    DONE_FAILURE
  };

  struct UiJob {
    std::string json;
    std::string state;
    double hold_s{0.0};

    rclcpp::Time t_start;
    rclcpp::Time t_phase_deadline;
    rclcpp::Time t_hold_end;

    rclcpp::Client<UiCommandSrv>::SharedFuture ui_future;
    bool ui_future_valid{false};

    uint64_t token{0};
  };

  void publish_ui_result(const std::string & v)
  {
    std_msgs::msg::String out;
    out.data = v;
    pub_ui_result_->publish(out);
  }

  void enqueue_ui_cmd(const std::string & raw_json)
  {
    const bool preempt = get_parameter("preempt_ui_on_new_cmd").as_bool();

    if (preempt) {
      pending_ui_.clear();
      pending_ui_.push_back(raw_json);

      if (phase_ != Phase::IDLE) {
        ++ui_token_;
        job_.ui_future_valid = false;
        phase_ = Phase::DONE_FAILURE;
      } else {
        start_next_ui_job();
      }
      return;
    }

    pending_ui_.push_back(raw_json);
    if (phase_ == Phase::IDLE) {
      start_next_ui_job();
    }
  }

  void start_next_ui_job()
  {
    if (pending_ui_.empty()) {
      phase_ = Phase::IDLE;
      return;
    }

    UiJob j;
    j.json = pending_ui_.front();
    pending_ui_.pop_front();

    j.token = ++ui_token_;
    j.state = to_lower(json_get_string(j.json, "state").value_or("default"));

    if (j.json.find("\"hold\"") != std::string::npos) {
      j.hold_s = json_get_number(j.json, "hold", 0.0);
    } else if (j.json.find("\"duration\"") != std::string::npos) {
      j.hold_s = json_get_number(j.json, "duration", 0.0);
    } else {
      j.hold_s = 0.0;
    }

    j.t_start = now_time(this);
    job_ = j;

    phase_ = Phase::WAIT_UI_SERVICE;
    job_.t_phase_deadline =
      job_.t_start + rclcpp::Duration::from_seconds(get_parameter("ui_wait_service_s").as_double());

    publish_ui_result("running");
  }

  void fail_ui_job_and_continue()
  {
    publish_ui_result("failure");
    job_.ui_future_valid = false;
    phase_ = Phase::DONE_FAILURE;
  }

  void succeed_ui_job_and_continue()
  {
    publish_ui_result("success");
    job_.ui_future_valid = false;
    phase_ = Phase::DONE_SUCCESS;
  }

  void tick()
  {
    switch (phase_) {
      case Phase::IDLE:
        return;

      case Phase::DONE_SUCCESS:
      case Phase::DONE_FAILURE:
        start_next_ui_job();
        return;

      case Phase::WAIT_UI_SERVICE: {
        publish_ui_result("running");

        if (ui_client_->service_is_ready()) {
          auto req = std::make_shared<UiCommandSrv::Request>();
          req->json = job_.json;

          job_.ui_future = ui_client_->async_send_request(req);
          job_.ui_future_valid = true;

          phase_ = Phase::WAIT_UI_FUTURE;
          job_.t_phase_deadline =
            now_time(this) +
            rclcpp::Duration::from_seconds(get_parameter("ui_future_timeout_s").as_double());
          return;
        }

        if (now_time(this) > job_.t_phase_deadline) {
          RCLCPP_ERROR(get_logger(), "UI service /robot/ui/command not available (timeout).");
          fail_ui_job_and_continue();
          return;
        }
        return;
      }

      case Phase::WAIT_UI_FUTURE: {
        publish_ui_result("running");

        if (!job_.ui_future_valid) {
          fail_ui_job_and_continue();
          return;
        }

        if (job_.ui_future.wait_for(0s) == std::future_status::ready) {
          auto resp = job_.ui_future.get();
          if (!resp->success) {
            RCLCPP_ERROR(get_logger(), "UI service response: success=false msg='%s'", resp->message.c_str());
            fail_ui_job_and_continue();
            return;
          }

          if (job_.hold_s > 0.0) {
            phase_ = Phase::HOLD_RUNNING;
            job_.t_hold_end = now_time(this) + rclcpp::Duration::from_seconds(job_.hold_s);
          } else {
            succeed_ui_job_and_continue();
          }
          return;
        }

        if (now_time(this) > job_.t_phase_deadline) {
          RCLCPP_ERROR(get_logger(), "UI service call timeout.");
          fail_ui_job_and_continue();
          return;
        }
        return;
      }

      case Phase::HOLD_RUNNING: {
        publish_ui_result("running");
        if (now_time(this) >= job_.t_hold_end) {
          succeed_ui_job_and_continue();
        }
        return;
      }

      default:
        return;
    }
  }

  // ---------------- LEDs pipeline (from /hri/leds_cmd) ----------------

  void cancel_active_led_goal()
  {
    if (!active_led_goal_) return;
    led_client_->async_cancel_goal(active_led_goal_);
    active_led_goal_.reset();
  }

  void send_led_goal_fire_and_forget(const LedsPlay::Goal & goal_msg)
  {
    rclcpp_action::Client<LedsPlay>::SendGoalOptions opts;
    opts.goal_response_callback = [](GoalHandleLedsPlay::SharedPtr) {};
    opts.result_callback = [](const GoalHandleLedsPlay::WrappedResult &) {};
    led_client_->async_send_goal(goal_msg, opts);
  }

  void on_leds_cmd(const std::string & raw)
  {
    std::string name = raw;
    while (!name.empty() && (name.back() == '\n' || name.back() == '\r' || name.back() == ' ' || name.back() == '\t')) {
      name.pop_back();
    }
    while (!name.empty() && (name.front() == ' ' || name.front() == '\t')) {
      name.erase(name.begin());
    }
    if (name.empty()) return;

    if (!ends_with(name, ".yml") && !ends_with(name, ".yaml")) {
      name += ".yml";
    }

    cancel_active_led_goal();

    const double wait_action = get_parameter("led_wait_action_s").as_double();
    if (!led_client_->wait_for_action_server(std::chrono::duration<double>(wait_action))) {
      RCLCPP_WARN(get_logger(), "LED action server not available (timeout). Dropping leds_cmd='%s'", name.c_str());
      return;
    }

    const std::string pkg = get_parameter("profiles_pkg").as_string();
    const std::string rel = get_parameter("profiles_rel_dir").as_string();

    std::string resolve_err;
    auto dir_opt = nao_ui_utils::resolve_in_share(pkg, rel, resolve_err);
    if (!dir_opt) {
      RCLCPP_WARN(get_logger(), "LED profiles not resolved: %s", resolve_err.c_str());
      return;
    }

    const std::string yaml_path = *dir_opt + "/" + name;

    std::string load_err;
    auto profile_opt = nao_ui_utils::load_profile_from_yaml_file(yaml_path, load_err);
    if (!profile_opt) {
      RCLCPP_WARN(get_logger(), "LED profile load failed '%s': %s", yaml_path.c_str(), load_err.c_str());
      return;
    }

    auto goal_msg = to_goal(*profile_opt);

    // NEW: "scene reset" -> turn off effectors not in this profile.
    if (get_parameter("led_scene_reset_others").as_bool()) {
      // Effectors are 0..NUMLEDS-1
      for (uint8_t eff = 0; eff < LedIndexes::NUMLEDS; ++eff) {
        if (!eff_in_targets(eff, goal_msg.leds)) {
          send_led_goal_fire_and_forget(make_off_goal(eff));
        }
      }
    }

    rclcpp_action::Client<LedsPlay>::SendGoalOptions opts;
    opts.goal_response_callback = [this](GoalHandleLedsPlay::SharedPtr gh) {
      if (!gh) {
        RCLCPP_WARN(get_logger(), "LED goal rejected");
        return;
      }
      active_led_goal_ = gh;
    };
    opts.result_callback = [](const GoalHandleLedsPlay::WrappedResult &) {};

    led_client_->async_send_goal(goal_msg, opts);
  }

  // ---------------- ROS entities ----------------
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_ui_result_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_ui_cmd_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_leds_cmd_;

  rclcpp::Client<UiCommandSrv>::SharedPtr ui_client_;
  rclcpp_action::Client<LedsPlay>::SharedPtr led_client_;

  rclcpp::TimerBase::SharedPtr tick_timer_;

  std::deque<std::string> pending_ui_;
  UiJob job_;
  Phase phase_{Phase::IDLE};
  uint64_t ui_token_{0};

  GoalHandleLedsPlay::SharedPtr active_led_goal_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UiStub>());
  rclcpp::shutdown();
  return 0;
}
