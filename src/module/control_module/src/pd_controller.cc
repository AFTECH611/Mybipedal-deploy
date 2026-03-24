#include "control_module/pd_controller.h"
#include <iostream>
#include <fstream>

namespace mybipedal_deploy::rl_control_module {

PDController::PDController(const bool use_sim_handles)
    : ControllerBase(use_sim_handles) {}

void PDController::Init(const YAML::Node &cfg_node) {
  // 初始化 joint_names_
  joint_names_.clear();
  joint_names_ = cfg_node["joint_list"].as<std::vector<std::string>>();
  // printf("joint_names_: ");
  // for (const auto &name : joint_names_) {
  //   printf("%s ", name.c_str());
  // }
  // printf("\n");
  joint_state_data_.name = joint_names_;
  joint_state_data_.position.resize(joint_names_.size(), 0.0);
  joint_state_data_.velocity.resize(joint_names_.size(), 0.0);
  joint_state_data_.effort.resize(joint_names_.size(), 0.0);
  start_joint_angles_.resize(joint_names_.size(), 0.0);

  // 初始化 joint_conf_
  joint_conf_.init_state = Eigen::Map<vector_t>(cfg_node["init_state"].as<std::vector<double>>().data(), cfg_node["init_state"].as<std::vector<double>>().size());
  joint_conf_.stiffness = Eigen::Map<vector_t>(cfg_node["stiffness"].as<std::vector<double>>().data(), cfg_node["stiffness"].as<std::vector<double>>().size());
  joint_conf_.damping = Eigen::Map<vector_t>(cfg_node["damping"].as<std::vector<double>>().data(), cfg_node["damping"].as<std::vector<double>>().size());
  // std::cout << "init_state: " << joint_conf_.init_state.transpose() << std::endl;
  // std::cout << "stiffness: " << joint_conf_.stiffness.transpose() << std::endl;
  // std::cout << "damping: " << joint_conf_.damping.transpose() << std::endl;

  // 其他 PD 参数
  if (cfg_node["is_keep_controller"]) {
    is_keep_controller_ = cfg_node["is_keep_controller"].as<bool>();
  }

}

void PDController::RestartController() {
  {
    std::lock_guard<std::shared_mutex> lock(joint_state_mutex_);
    start_joint_angles_ = joint_state_data_.position;
  }
  trans_mode_percent_ = 0.0;
}

void PDController::Update() {
  if (is_keep_controller_) {
    return;
  }

  trans_mode_percent_ += 1.0 / trans_mode_duration_s_ / 1000;
  trans_mode_percent_ = std::min(trans_mode_percent_, scalar_t(1));
}

my_ros2_proto::msg::JointCommand PDController::GetJointCmdData() {
  my_ros2_proto::msg::JointCommand joint_cmd;
  joint_cmd.name = joint_names_;
  joint_cmd.position.resize(joint_names_.size());
  joint_cmd.velocity.resize(joint_names_.size());
  joint_cmd.effort.resize(joint_names_.size());
  joint_cmd.damping.resize(joint_names_.size());
  joint_cmd.stiffness.resize(joint_names_.size());

  for (int ii = 0; ii < joint_names_.size(); ii++) {
    double pos_des;
    if (is_keep_controller_) {
      pos_des = start_joint_angles_[ii];
    } else {
      pos_des = start_joint_angles_[ii] * (1 - trans_mode_percent_) + joint_conf_.init_state(ii) * trans_mode_percent_;
    }

    joint_cmd.position[ii] = pos_des;
    joint_cmd.velocity[ii] = 0.0;
    joint_cmd.effort[ii] = 0.0;
    joint_cmd.stiffness[ii] = joint_conf_.stiffness(ii);
    joint_cmd.damping[ii] = joint_conf_.damping(ii);
  }
  return joint_cmd;
}

}  // namespace mybipedal_deploy::rl_control_module
