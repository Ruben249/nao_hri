#ifndef NAO_UI_UTILS__LED_PROFILE_HPP_
#define NAO_UI_UTILS__LED_PROFILE_HPP_

#include <array>
#include <cstdint>
#include "rosidl_runtime_cpp/message_initialization.hpp"


#include "std_msgs/msg/color_rgba.hpp"

namespace nao_ui_utils
{

struct LedProfile
{
  std::array<uint8_t, 2> leds{{0u, 0u}};
  uint8_t mode{0u};
  float frequency{0.0f};
  float duration{-1.0f};
  std::array<std_msgs::msg::ColorRGBA, 8> colors{
    std_msgs::msg::ColorRGBA(rosidl_runtime_cpp::MessageInitialization::ZERO)
  };

  std::array<float, 12> intensities{};
};

}  // namespace nao_ui_utils

#endif  // NAO_UI_UTILS__LED_PROFILE_HPP_
