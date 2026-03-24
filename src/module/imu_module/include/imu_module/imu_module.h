// Copyright (c) 2026, Mybipedal.
// All rights reserved.
#pragma once
#include <thread>
#include "aimrt_module_cpp_interface/module_base.h"
#include "dm_imu_l1/include/dm_imu/imu_driver.h"

namespace mybipedal_deploy::imu_module {

class ImuModule : public aimrt::ModuleBase {
 public:
  ImuModule() = default;
  ~ImuModule() override = default;

  [[nodiscard]] aimrt::ModuleInfo Info() const override {
    return aimrt::ModuleInfo{.name = "ImuModule"};
  }
  bool Initialize(aimrt::CoreRef core) override;
  bool Start() override;
  void Shutdown() override;

 private:
  auto GetLogger() { return core_.GetLogger(); }

 private:
  aimrt::CoreRef core_;
  std::shared_ptr<dm_imu::ImuDriver> imu_;

  std::string port_;
  int    baud_{921600};
  bool   do_config_{true};
  double publish_frequency_{1000.0};

  aimrt::channel::PublisherRef pub_imu_;
};

}  // namespace mybipedal_deploy::imu_module