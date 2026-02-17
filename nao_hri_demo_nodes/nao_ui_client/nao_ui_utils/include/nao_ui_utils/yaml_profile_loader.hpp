#ifndef NAO_UI_UTILS__YAML_PROFILE_LOADER_HPP_
#define NAO_UI_UTILS__YAML_PROFILE_LOADER_HPP_

#include <optional>
#include <string>
#include <vector>

#include "nao_ui_utils/led_profile.hpp"

namespace nao_ui_utils
{

std::vector<std::string> list_yaml_files(const std::string & dir, bool skip_non_emotion_files = true);

std::optional<LedProfile> load_profile_from_yaml_file(
  const std::string & yaml_path,
  std::string & error);

std::optional<std::string> resolve_in_share(
  const std::string & package_name,
  const std::string & relative_path,
  std::string & error);

}  // namespace nao_ui_utils

#endif  // NAO_UI_UTILS__YAML_PROFILE_LOADER_HPP_
