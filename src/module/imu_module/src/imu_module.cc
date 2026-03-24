#include "imu_module/imu_module.h"
#include <yaml-cpp/yaml.h>
#include "aimrt_module_ros2_interface/channel/ros2_channel.h"
#include <sensor_msgs/msg/imu.hpp>

namespace mybipedal_deploy::imu_module {

bool ImuModule::Initialize(aimrt::CoreRef core) {
  core_ = core;
  try {
    auto file_path = core_.GetConfigurator().GetConfigFilePath();
    YAML::Node cfg_node = YAML::LoadFile(file_path.data());
    port_              = cfg_node["port"].as<std::string>();
    baud_              = cfg_node["baud"].as<int>();
    do_config_         = cfg_node["do_config"].as<bool>();
    publish_frequency_ = cfg_node["publish_frequency"].as<double>();

    // Tạo driver — chưa open hardware
    imu_ = std::make_shared<dm_imu::ImuDriver>(port_, baud_);

    pub_imu_ = core_.GetChannelHandle().GetPublisher("/imu/data");
    aimrt::channel::RegisterPublishType<sensor_msgs::msg::Imu>(pub_imu_);

    AIMRT_INFO("Init succeeded.");
    return true;
  } catch (const std::exception& e) {
    AIMRT_ERROR("Init failed, {}", e.what());
    return false;
  }
}

bool ImuModule::Start() {
  // Dùng PUSH callback — không cần publish_thread_ riêng
  aimrt::channel::PublisherProxy<sensor_msgs::msg::Imu> pub(pub_imu_);
    
  uint32_t every_n = static_cast<uint32_t>(1000.0 / publish_frequency_);
  if (every_n < 1) every_n = 1;  // giới hạn tối đa 1000Hz

  imu_->setObsCallback([this, pub](const dm_imu::ImuObservation& obs) mutable {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp.sec     = obs.timestamp_ns / 1'000'000'000ULL;
    msg.header.stamp.nanosec = obs.timestamp_ns % 1'000'000'000ULL;
    msg.header.frame_id      = "imu_link";

    msg.orientation.w = obs.qw;   // ✅ đúng field name
    msg.orientation.x = obs.qx;
    msg.orientation.y = obs.qy;
    msg.orientation.z = obs.qz;

    msg.angular_velocity.x = obs.gyr_x;  // ✅
    msg.angular_velocity.y = obs.gyr_y;
    msg.angular_velocity.z = obs.gyr_z;

    msg.linear_acceleration.x = obs.acc_x;  // ✅
    msg.linear_acceleration.y = obs.acc_y;
    msg.linear_acceleration.z = obs.acc_z;

    pub.Publish(msg);
  }, every_n);

  if (!imu_->open(do_config_)) {
    AIMRT_ERROR("Cannot open IMU on port {}", port_);
    return false;
  }

  AIMRT_INFO("Started succeeded.");
  return true;
}

void ImuModule::Shutdown() {
  if (imu_) {
    imu_->setObsCallback(nullptr, 1);
    imu_->close();
    imu_.reset();
  }
}

}  // namespace mybipedal_deploy::imu_module