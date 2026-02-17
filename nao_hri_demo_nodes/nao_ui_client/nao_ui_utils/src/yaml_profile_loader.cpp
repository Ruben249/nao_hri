#include "nao_ui_utils/yaml_profile_loader.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <unordered_set>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nao_led_interfaces/msg/led_indexes.hpp"
#include "nao_led_interfaces/msg/led_modes.hpp"

namespace fs = std::filesystem;

namespace nao_ui_utils
{

static inline std::string trim(const std::string & s)
{
  // Trim leading and trailing whitespace from the string
  size_t b = 0;
  while (b < s.size() && std::isspace(static_cast<unsigned char>(s[b]))) {++b;}
  size_t e = s.size();
  while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) {--e;}
  return s.substr(b, e - b);
}

static inline std::string strip_comment(const std::string & s)
{
  // Remove comments from the line. A comment starts with '#' and continues to the end of the line.
  const auto pos = s.find('#');
  if (pos == std::string::npos) {return s;}
  return s.substr(0, pos);
}

static bool starts_with(const std::string & s, const std::string & p)
{
  // Check if string s starts with prefix p
  return s.rfind(p, 0) == 0;
}

static std::vector<std::string> split_csv(const std::string & inside)
{
  // Split the string by commas and trim whitespace from each element
  std::vector<std::string> out;
  std::stringstream ss(inside);
  std::string tok;
  while (std::getline(ss, tok, ',')) {
    tok = trim(tok);
    if (!tok.empty()) {out.push_back(tok);}
  }
  return out;
}

static std::optional<std::string> bracket_content(const std::string & line)
{
  // Extract the content inside the first pair of brackets [..] in the line
  const auto lb = line.find('[');
  const auto rb = line.rfind(']');
  if (lb == std::string::npos || rb == std::string::npos || rb <= lb) {
    return std::nullopt;
  }
  return line.substr(lb + 1, rb - lb - 1);
}

static std::optional<float> to_float(const std::string & s)
{
  // Convert string to float, return nullopt if conversion fails
  try {
    return std::stof(trim(s));
  } catch (...) {
    return std::nullopt;
  }
}

static std::optional<uint8_t> parse_mode(const std::string & v)
{
  // Parse the mode string and return the corresponding uint8_t value from LedModes
  const auto val = trim(v);
  if (val == "STEADY") {return nao_led_interfaces::msg::LedModes::STEADY;}
  if (val == "BLINKING") {return nao_led_interfaces::msg::LedModes::BLINKING;}
  if (val == "LOOP") {return nao_led_interfaces::msg::LedModes::LOOP;}
  return std::nullopt;
}

static std::optional<uint8_t> parse_led_index(const std::string & v)
{
  // Parse the LED index string and return the corresponding uint8_t value from LedIndexes
  const auto val = trim(v);
  if (val == "HEAD") {return nao_led_interfaces::msg::LedIndexes::HEAD;}
  if (val == "REYE") {return nao_led_interfaces::msg::LedIndexes::REYE;}
  if (val == "LEYE") {return nao_led_interfaces::msg::LedIndexes::LEYE;}
  if (val == "REAR") {return nao_led_interfaces::msg::LedIndexes::REAR;}
  if (val == "LEAR") {return nao_led_interfaces::msg::LedIndexes::LEAR;}
  if (val == "CHEST") {return nao_led_interfaces::msg::LedIndexes::CHEST;}
  if (val == "RFOOT") {return nao_led_interfaces::msg::LedIndexes::RFOOT;}
  if (val == "LFOOT") {return nao_led_interfaces::msg::LedIndexes::LFOOT;}
  return std::nullopt;
}

std::vector<std::string> list_yaml_files(const std::string & dir, bool skip_non_emotion_files)
{
  // List all YAML files in the directory, optionally skipping non-emotion ones based on their names
  std::vector<std::string> files;
  if (!fs::exists(dir)) {
    return files;
  }

  // If skipping non-emotion files, we define a set of names to skip. This is a simple 
  // heuristic based on the filename.
  const std::unordered_set<std::string> skip_names = {"default", "comparision", "comparison"};
  for (const auto & entry : fs::directory_iterator(dir)) {
    if (!entry.is_regular_file()) {continue;}
    const auto ext = entry.path().extension().string();
    if (ext != ".yml" && ext != ".yaml") {continue;}

    if (skip_non_emotion_files) {
      const auto stem = entry.path().stem().string();
      if (skip_names.count(stem) > 0) {continue;}
    }
    files.push_back(entry.path().string());
  }

  std::sort(files.begin(), files.end());
  return files;
}

std::optional<std::string> resolve_in_share(
  const std::string & package_name,
  const std::string & relative_path,
  std::string & error)
{
  // Resolve the absolute path of a file in the share directory of a ROS2 package
  try {
    const auto share = ament_index_cpp::get_package_share_directory(package_name);
    const fs::path p = fs::path(share) / relative_path;
    if (!fs::exists(p)) {
      error = "No existe el path en share: " + p.string();
      return std::nullopt;
    }
    return p.string();
  } catch (const std::exception & e) {
    error = std::string("Error resolviendo share dir: ") + e.what();
    return std::nullopt;
  }
}

std::optional<LedProfile> load_profile_from_yaml_file(const std::string & yaml_path, std::string & error)
{
  // Load an LED profile from a YAML file. The YAML file is expected to have a specific format that 
  // defines the fields of LedProfile.
  std::ifstream ifs(yaml_path);
  if (!ifs.is_open()) {
    error = "No se pudo abrir YAML: " + yaml_path;
    return std::nullopt;
  }

  LedProfile p;

  for (auto & c : p.colors) {
    c.r = 0.0f; c.g = 0.0f; c.b = 0.0f; c.a = 1.0f;
  }
  p.intensities.fill(0.0f);

  bool in_colors = false;
  size_t color_idx = 0;

  std::string line;
  /*
  While reading the YAML file line by line, we trim whitespace and strip comments. 
  We look for specific keys like "mode", "frequency", "duration", "leds", "intensities", and a 
  special section "colors". For the "colors" section, we expect lines starting with "-" that 
  define RGBA values in a list format. We parse these values and fill the LedProfile struct accordingly. 
  If any parsing error occurs, we set the error message and return nullopt.
  */
  while (std::getline(ifs, line)) {
    line = trim(strip_comment(line));
    if (line.empty()) {continue;}

    if (starts_with(line, "colors:")) {
      in_colors = true;
      continue;
    }

    if (in_colors) {
      if (starts_with(line, "-")) {
        const auto b = bracket_content(line);
        if (!b) {
          error = "Formato colors inválido en: " + yaml_path;
          return std::nullopt;
        }
        const auto vals = split_csv(*b);
        if (vals.size() != 4) {
          error = "colors requiere 4 floats [r,g,b,a] en: " + yaml_path;
          return std::nullopt;
        }
        if (color_idx >= p.colors.size()) {
          continue;
        }
        auto r = to_float(vals[0]);
        auto g = to_float(vals[1]);
        auto b2 = to_float(vals[2]);
        auto a = to_float(vals[3]);
        if (!r || !g || !b2 || !a) {
          error = "No se pudieron parsear floats en colors en: " + yaml_path;
          return std::nullopt;
        }
        p.colors[color_idx].r = *r;
        p.colors[color_idx].g = *g;
        p.colors[color_idx].b = *b2;
        p.colors[color_idx].a = *a;
        color_idx++;
        continue;
      } else {
        in_colors = false;
      }
    }

    const auto colon = line.find(':');

    // If there is no colon, it's an invalid line for our purposes, so we skip it
    if (colon == std::string::npos) {continue;}

    const auto key = trim(line.substr(0, colon));
    const auto val = trim(line.substr(colon + 1));

    if (key == "mode") {
      const auto m = parse_mode(val);
      if (!m) {
        error = "mode inválido en: " + yaml_path;
        return std::nullopt;
      }
      p.mode = *m;
    } else if (key == "frequency") {
      const auto f = to_float(val);
      if (!f) {
        error = "frequency inválido en: " + yaml_path;
        return std::nullopt;
      }
      p.frequency = *f;
    } else if (key == "duration") {
      const auto d = to_float(val);
      if (!d) {
        error = "duration inválido en: " + yaml_path;
        return std::nullopt;
      }
      p.duration = *d;
    } else if (key == "leds") {
      const auto b = bracket_content(val);
      if (!b) {
        error = "leds inválido (esperado [..]) en: " + yaml_path;
        return std::nullopt;
      }
      const auto leds = split_csv(*b);
      if (leds.empty() || leds.size() > 2) {
        error = "leds debe tener 1 o 2 elementos en: " + yaml_path;
        return std::nullopt;
      }
      p.leds = {0u, 0u};
      for (size_t i = 0; i < leds.size(); ++i) {
        const auto li = parse_led_index(leds[i]);
        if (!li) {
          error = "led index inválido '" + leds[i] + "' en: " + yaml_path;
          return std::nullopt;
        }
        p.leds[i] = *li;
      }
    } else if (key == "intensities") {
      const auto b = bracket_content(val);
      if (!b) {
        error = "intensities inválido (esperado [..]) en: " + yaml_path;
        return std::nullopt;
      }
      const auto ints = split_csv(*b);
      if (ints.size() != p.intensities.size()) {
        error = "intensities debe tener 12 floats en: " + yaml_path;
        return std::nullopt;
      }
      for (size_t i = 0; i < ints.size(); ++i) {
        const auto f = to_float(ints[i]);
        if (!f) {
          error = "intensity inválida en: " + yaml_path;
          return std::nullopt;
        }
        p.intensities[i] = *f;
      }
    }
  }

  return p;
}

}  // namespace nao_ui_utils
