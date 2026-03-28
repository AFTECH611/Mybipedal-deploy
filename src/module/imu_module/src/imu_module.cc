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
    if (!imu_->open(do_config_)) {
      AIMRT_ERROR("Cannot open IMU on port {}", port_);
      return false;
    }
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
  is_running_ = true;
  publish_thread_ = std::thread(&ImuModule::PublishLoop, this);

  return true;
}

void ImuModule::Shutdown() {
  is_running_ = false;
  if (imu_) {
    imu_->setObsCallback(nullptr, 1);
    imu_->close();
    imu_.reset();
  }
}

void ImuModule::PublishLoop() {
  aimrt::channel::PublisherProxy<sensor_msgs::msg::Imu> pub_imu(pub_imu_);
  sensor_msgs::msg::Imu imu_msg;
  auto period = std::chrono::nanoseconds((uint64_t)(1 / publish_frequency_ * 1000000000));
  auto next_loop_time = std::chrono::steady_clock::now();
  while (is_running_) {
    // get time
    timeval now;
    gettimeofday(&now, NULL);

    builtin_interfaces::msg::Time stamp;
    stamp.sec = now.tv_sec;
    stamp.nanosec = now.tv_usec * 1000;
    dm_imu::ImuObservation imu = imu_->getObs();
    imu_msg.angular_velocity.x = imu.gyr_x;
    imu_msg.angular_velocity.y = imu.gyr_y;
    imu_msg.angular_velocity.z = imu.gyr_z;
    imu_msg.linear_acceleration.x = imu.acc_x;
    imu_msg.linear_acceleration.y = imu.acc_y;
    imu_msg.linear_acceleration.z = imu.acc_z;
    imu_msg.orientation.w = imu.qw;
    imu_msg.orientation.x = imu.qx;
    imu_msg.orientation.y = imu.qy;
    imu_msg.orientation.z = imu.qz;
    imu_msg.header.stamp = stamp;
    pub_imu.Publish(imu_msg);
    // AIMRT_INFO("Publish imu data, linear_acceleration: [{:.5f}, {:.5f}, {:.5f}]", imu.acc_x, imu.acc_y, imu.acc_z);

    next_loop_time += period;
    std::this_thread::sleep_until(next_loop_time);
  }
}
}  // namespace mybipedal_deploy::imu_module