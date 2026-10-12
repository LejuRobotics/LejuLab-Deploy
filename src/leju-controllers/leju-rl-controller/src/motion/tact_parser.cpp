#include "leju-rl-controller/motion/tact_parser.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

#include <nlohmann/json.hpp>

namespace leju {
namespace vr {
namespace tact_player {
namespace {

constexpr double kKeyframeToSec = 0.01;

bool IsNumber(const nlohmann::json& v) {
  return v.is_number_float() || v.is_number_integer() || v.is_number_unsigned();
}

bool ReadNumber(const nlohmann::json& obj,
                const char* key,
                double* out,
                std::string* err) {
  if (!obj.contains(key)) {
    *err = std::string("missing required field: ") + key;
    return false;
  }
  const auto& value = obj[key];
  if (!IsNumber(value)) {
    *err = std::string("field '") + key + "' must be number";
    return false;
  }
  *out = value.get<double>();
  return true;
}

}  // namespace

bool LoadTactFile(const std::string& path,
                  const ParseOptions& options,
                  TactAction* action,
                  std::string* error_message) {
  if (action == nullptr || error_message == nullptr) {
    return false;
  }

  std::ifstream input(path);
  if (!input.is_open()) {
    *error_message = "failed to open tact file: " + path;
    return false;
  }

  nlohmann::json root;
  try {
    input >> root;
  } catch (const std::exception& e) {
    *error_message = std::string("json parse failed: ") + e.what();
    return false;
  }

  if (!root.is_object()) {
    *error_message = "tact root must be json object";
    return false;
  }

  double robot_type = 0.0;
  double first_kf = 0.0;
  double finish_kf = 0.0;
  // robotType is optional, default to 0
  if (root.contains("robotType") && IsNumber(root["robotType"])) {
    robot_type = root["robotType"].get<double>();
  }
  if (!ReadNumber(root, "first", &first_kf, error_message) ||
      !ReadNumber(root, "finish", &finish_kf, error_message)) {
    return false;
  }

  if (!root.contains("frames") || !root["frames"].is_array()) {
    *error_message = "field 'frames' must be array";
    return false;
  }

  const auto& frames = root["frames"];
  if (frames.empty()) {
    *error_message = "frames cannot be empty";
    return false;
  }

  TactAction parsed;
  parsed.robot_type = static_cast<int>(std::round(robot_type));
  parsed.first_sec = first_kf * kKeyframeToSec;
  parsed.finish_sec = finish_kf * kKeyframeToSec;
  parsed.has_waist = (options.waist_index >= 0);
  parsed.has_hand = (options.hand_start_index >= 0);

  for (std::size_t frame_idx = 0; frame_idx < frames.size(); ++frame_idx) {
    const auto& frame_json = frames[frame_idx];
    if (!frame_json.is_object()) {
      *error_message = "frame[" + std::to_string(frame_idx) + "] must be object";
      return false;
    }

    if (!frame_json.contains("servos") || !frame_json["servos"].is_array()) {
      *error_message = "frame[" + std::to_string(frame_idx) + "].servos must be array";
      return false;
    }
    if (!frame_json.contains("attribute") || !frame_json["attribute"].is_object()) {
      *error_message = "frame[" + std::to_string(frame_idx) + "].attribute must be object";
      return false;
    }

    double keyframe = 0.0;
    if (!ReadNumber(frame_json, "keyframe", &keyframe, error_message)) {
      *error_message = "frame[" + std::to_string(frame_idx) + "]." + *error_message;
      return false;
    }
    if (keyframe < 0.0) {
      *error_message = "frame[" + std::to_string(frame_idx) + "].keyframe must be non-negative";
      return false;
    }

    const auto& servos_json = frame_json["servos"];
    if (servos_json.size() < options.arm_dof) {
      *error_message = "frame[" + std::to_string(frame_idx) + "].servos size " +
                       std::to_string(servos_json.size()) +
                       " < arm_dof " + std::to_string(options.arm_dof);
      return false;
    }

    TactFrame frame;
    frame.keyframe = keyframe;
    frame.servos_deg.resize(options.arm_dof, 0.0);
    frame.attributes.resize(options.arm_dof);

    for (std::size_t joint = 0; joint < options.arm_dof; ++joint) {
      const auto& servo = servos_json[joint];
      if (!IsNumber(servo)) {
        *error_message = "frame[" + std::to_string(frame_idx) + "].servos[" +
                         std::to_string(joint) + "] must be number";
        return false;
      }
      frame.servos_deg[joint] = servo.get<double>();

      const std::string key = std::to_string(joint + 1);
      const auto& attrs = frame_json["attribute"];
      if (!attrs.contains(key) || !attrs[key].is_object()) {
        *error_message = "frame[" + std::to_string(frame_idx) + "].attribute['" + key + "'] missing";
        return false;
      }

      const auto& attr = attrs[key];
      if (!attr.contains("CP") || !attr["CP"].is_array() || attr["CP"].size() != 2) {
        *error_message = "frame[" + std::to_string(frame_idx) + "].attribute['" + key + "'].CP invalid";
        return false;
      }

      for (std::size_t cp_idx = 0; cp_idx < 2; ++cp_idx) {
        const auto& cp = attr["CP"][cp_idx];
        if (!cp.is_array() || cp.size() != 2 || !IsNumber(cp[0]) || !IsNumber(cp[1])) {
          *error_message = "frame[" + std::to_string(frame_idx) + "].attribute['" + key + "'].CP[" +
                           std::to_string(cp_idx) + "] invalid";
          return false;
        }
        frame.attributes[joint].cp[cp_idx][0] = cp[0].get<double>();
        frame.attributes[joint].cp[cp_idx][1] = cp[1].get<double>();
      }

      if (attr.contains("CPType") && attr["CPType"].is_array() && attr["CPType"].size() == 2) {
        for (std::size_t type_idx = 0; type_idx < 2; ++type_idx) {
          if (attr["CPType"][type_idx].is_string()) {
            frame.attributes[joint].cp_type[type_idx] = attr["CPType"][type_idx].get<std::string>();
          }
        }
      }
    }

    // Parse waist joint if configured
    if (options.waist_index >= 0) {
      const std::size_t waist_idx = static_cast<std::size_t>(options.waist_index);
      if (waist_idx < servos_json.size()) {
        const auto& waist_servo = servos_json[waist_idx];
        if (IsNumber(waist_servo)) {
          frame.has_waist = true;
          frame.waist_deg = waist_servo.get<double>();

          // Parse waist attribute (CP)
          const std::string waist_key = std::to_string(waist_idx + 1);
          const auto& attrs = frame_json["attribute"];
          if (attrs.contains(waist_key) && attrs[waist_key].is_object()) {
            const auto& waist_attr = attrs[waist_key];
            if (waist_attr.contains("CP") && waist_attr["CP"].is_array() && waist_attr["CP"].size() == 2) {
              for (std::size_t cp_idx = 0; cp_idx < 2; ++cp_idx) {
                const auto& cp = waist_attr["CP"][cp_idx];
                if (cp.is_array() && cp.size() == 2 && IsNumber(cp[0]) && IsNumber(cp[1])) {
                  frame.waist_attribute.cp[cp_idx][0] = cp[0].get<double>();
                  frame.waist_attribute.cp[cp_idx][1] = cp[1].get<double>();
                }
              }
            }
            if (waist_attr.contains("CPType") && waist_attr["CPType"].is_array() && waist_attr["CPType"].size() == 2) {
              for (std::size_t type_idx = 0; type_idx < 2; ++type_idx) {
                if (waist_attr["CPType"][type_idx].is_string()) {
                  frame.waist_attribute.cp_type[type_idx] = waist_attr["CPType"][type_idx].get<std::string>();
                }
              }
            }
          }
        }
      }
    }


    // Parse hand data if configured (12 values: left 6 + right 6).
    if (options.hand_start_index >= 0) {
      const std::size_t hand_start = static_cast<std::size_t>(options.hand_start_index);
      if (servos_json.size() >= hand_start + 12) {
        frame.has_hand = true;
        frame.hand_positions.resize(12, 0.0);
        frame.hand_attributes.resize(12);

        for (std::size_t h = 0; h < 12; ++h) {
          const std::size_t servo_idx = hand_start + h;
          const auto& servo = servos_json[servo_idx];
          if (!IsNumber(servo)) {
            *error_message = "frame[" + std::to_string(frame_idx) + "].servos[" +
                             std::to_string(servo_idx) + "] must be number";
            return false;
          }
          frame.hand_positions[h] = servo.get<double>();

          const std::string key = std::to_string(servo_idx + 1);
          const auto& attrs = frame_json["attribute"];
          if (attrs.contains(key) && attrs[key].is_object()) {
            const auto& attr = attrs[key];
            if (attr.contains("CP") && attr["CP"].is_array() && attr["CP"].size() == 2) {
              for (std::size_t cp_idx = 0; cp_idx < 2; ++cp_idx) {
                const auto& cp = attr["CP"][cp_idx];
                if (cp.is_array() && cp.size() == 2 && IsNumber(cp[0]) && IsNumber(cp[1])) {
                  frame.hand_attributes[h].cp[cp_idx][0] = cp[0].get<double>();
                  frame.hand_attributes[h].cp[cp_idx][1] = cp[1].get<double>();
                }
              }
            }
            if (attr.contains("CPType") && attr["CPType"].is_array() && attr["CPType"].size() == 2) {
              for (std::size_t type_idx = 0; type_idx < 2; ++type_idx) {
                if (attr["CPType"][type_idx].is_string()) {
                  frame.hand_attributes[h].cp_type[type_idx] = attr["CPType"][type_idx].get<std::string>();
                }
              }
            }
          }
        }
      }
    }

    parsed.frames.push_back(std::move(frame));
  }

  std::sort(parsed.frames.begin(), parsed.frames.end(), [](const TactFrame& a, const TactFrame& b) {
    return a.keyframe < b.keyframe;
  });

  // Add initial frame at keyframe=0 if needed
  if (options.add_init_frame && !parsed.frames.empty() && parsed.frames.front().keyframe > 0.0) {
    TactFrame init_frame;
    init_frame.keyframe = 0.0;
    init_frame.servos_deg.resize(options.arm_dof, 0.0);
    init_frame.attributes.resize(options.arm_dof);

    // Use provided init_arm_pos or copy from first frame
    if (options.init_arm_pos.size() >= options.arm_dof) {
      for (std::size_t j = 0; j < options.arm_dof; ++j) {
        init_frame.servos_deg[j] = options.init_arm_pos[j];
      }
    } else {
      // Use first frame values as initial position
      for (std::size_t j = 0; j < options.arm_dof; ++j) {
        init_frame.servos_deg[j] = parsed.frames.front().servos_deg[j];
      }
    }

    // Set control points to zero (no tangent influence)
    for (std::size_t j = 0; j < options.arm_dof; ++j) {
      init_frame.attributes[j].cp[0] = {0.0, 0.0};
      init_frame.attributes[j].cp[1] = {0.0, 0.0};
      init_frame.attributes[j].cp_type = {"AUTO", "AUTO"};
    }

    // Set waist initial frame if needed
    if (parsed.has_waist && parsed.frames.front().has_waist) {
      init_frame.has_waist = true;
      if (!options.init_waist_pos.empty()) {
        init_frame.waist_deg = options.init_waist_pos[0];
      } else {
        init_frame.waist_deg = parsed.frames.front().waist_deg;
      }
      init_frame.waist_attribute.cp[0] = {0.0, 0.0};
      init_frame.waist_attribute.cp[1] = {0.0, 0.0};
      init_frame.waist_attribute.cp_type = {"AUTO", "AUTO"};
    }


    // Set hand initial frame if needed
    if (parsed.frames.front().has_hand) {
      init_frame.has_hand = true;
      init_frame.hand_positions = parsed.frames.front().hand_positions;
      init_frame.hand_attributes.resize(12);
      for (std::size_t h = 0; h < 12; ++h) {
        init_frame.hand_attributes[h].cp[0] = {0.0, 0.0};
        init_frame.hand_attributes[h].cp[1] = {0.0, 0.0};
        init_frame.hand_attributes[h].cp_type = {"AUTO", "AUTO"};
      }
    }

    parsed.frames.insert(parsed.frames.begin(), init_frame);
  }

  if (parsed.finish_sec <= parsed.first_sec) {
    const double last_kf = parsed.frames.back().keyframe;
    parsed.finish_sec = std::max(parsed.finish_sec, last_kf * kKeyframeToSec);
  }

  *action = std::move(parsed);
  return true;
}

}  // namespace tact_player
}  // namespace vr
}  // namespace leju
