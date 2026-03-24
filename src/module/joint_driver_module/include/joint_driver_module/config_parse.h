// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

// projects
#include "util/log_util.h"
#include "yaml-cpp/yaml.h"

namespace YAML {

struct CanServiceConfig {
  int bind_cpu;
  int rt_priority;
  uint64_t cycle_time_ns;
};

template <>
struct convert<CanServiceConfig> {
  static bool decode(const Node& node, CanServiceConfig& rhs) {
    try {
      rhs.bind_cpu = node["bind_cpu"].as<int>();
      rhs.rt_priority = node["rt_priority"].as<int>();
      rhs.cycle_time_ns = node["cycle_time_ns"].as<uint64_t>();
      return true;
    } catch (const YAML::Exception& e) {
      auto lgr = aimrt::common::util::SimpleLogger();
      AIMRT_HL_ERROR(lgr, "Parse Can Service Config failed, {}", e.what());
      return false;
    }
  }
};

struct DriverConfig {
  std::string name;
  std::vector<std::string> interfaces;
  bool enable;
  struct Actuator {
    std::string name;
    std::string type;
    uint32_t can_id;
  };
  std::vector<Actuator> ch[4];
};
using DriverNetworkConfig = std::vector<DriverConfig>;

template <>
struct convert<DriverNetworkConfig> {
  static bool decode(const Node& node, DriverNetworkConfig& rhs) {
    try {
      for (auto& driver_node : node) {
        DriverConfig driver_cfg;
        driver_cfg.name = driver_node["name"].as<std::string>();
        driver_cfg.interfaces = driver_node["interface"].as<std::vector<std::string>>(); // ✅ Fix 1
        driver_cfg.enable = driver_node["enable"].as<bool>();
        std::vector<std::string> ch_names{"canbus_1", "canbus_2", "canbus_3", "canbus_4"};
        for (size_t i = 0; i < ch_names.size(); i++) {
          if (driver_node[ch_names[i]].IsDefined()) {
            for (auto& actr_node : driver_node[ch_names[i]]) {
              DriverConfig::Actuator actr_cfg;
              actr_cfg.name = actr_node["name"].as<std::string>();
              actr_cfg.type = actr_node["type"].as<std::string>();
              actr_cfg.can_id = actr_node["can_id"].as<uint32_t>();
              driver_cfg.ch[i].push_back(actr_cfg);
            }
          }
        }
        rhs.push_back(driver_cfg);
      }
      return true;
    } catch (const YAML::Exception& e) {
      auto lgr = aimrt::common::util::SimpleLogger();
      AIMRT_HL_ERROR(lgr, "Parse Driver Config failed, {}", e.what());
      return false;
    }
  }
};

}  // namespace YAML