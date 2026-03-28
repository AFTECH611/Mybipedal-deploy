// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

// cpp
#include <future>
#include <thread>
#include <unordered_map>

// projects
#include "aimrt_module_cpp_interface/module_base.h"
#include "joint_driver_module/config_parse.h"
#include "joint_driver_module/transmission.h"
#include "joint_driver_module/robstride_controller/robstride_api/include/xyber_controller.h"
#include "my_ros2_proto/msg/joint_command.hpp"

namespace mybipedal_deploy::joint_driver_module {

class JointDriverModule : public aimrt::ModuleBase {
 public:
  JointDriverModule() = default;
  ~JointDriverModule() override = default;

  bool Initialize(aimrt::CoreRef core) override;
  bool Start() override;
  void Shutdown() override;

 private:
  [[nodiscard]] aimrt::ModuleInfo Info() const override {
    return aimrt::ModuleInfo{.name = "JointDriverModule"};
  }

 private:
  bool InitDriver(YAML::Node& cfg_node);
  bool InitTransmission(YAML::Node& cfg_node);

  void PublishLoop();
  auto GetLogger() { return core_.GetLogger(); }

  void JointCmdCallback(const std::shared_ptr<const my_ros2_proto::msg::JointCommand>& msg);

 private:
  bool actuator_debug_ = false;
  bool enable_actuator_ = false;
  double publish_frequency_ = 100.0f;
  std::atomic_bool is_running_ = false;
  std::mutex rw_mtx_;
  std::thread publish_thread_;
  std::vector<std::string> joint_name_list_;
  std::vector<std::string> actuator_name_list_;
  std::unordered_map<std::string, DataSpace> joint_data_space_;
  std::unordered_map<std::string, DataSpace> actuator_data_space_;

  aimrt::CoreRef core_;
  aimrt::channel::PublisherRef pub_joint_state_;
  aimrt::channel::PublisherRef pub_actuator_cmd_;
  aimrt::channel::PublisherRef pub_actuator_state_;
  aimrt::channel::SubscriberRef sub_joint_cmd_;

  YAML::CanServiceConfig can_service_cfg_;
  YAML::DriverNetworkConfig driver_network_cfg_;
  TransimissionManager transmission_;
  xyber::XyberControllerPtr xyber_ctrl_;
};

}  // namespace mybipedal_deploy::joint_driver_module