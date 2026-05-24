#pragma once

// =============================================================================
//  oled_module.h  –  AimRT module that drives the SSD1306 OLED display
//
//  Architecture
//  ────────────
//  AimRT executor (oled_pub_thread)
//    └─ MainLoop()          – event-driven UI loop (200 ms tick)
//         ├─ pop EventQueue
//         ├─ handleEvent / renderUI
//         └─ auto-sleep check
//
//  std::thread  gpio_thread_  [SCHED_OTHER, lowest priority]
//    └─ polls encoder CLK/DT + SW + back-button via libgpiod
//       pushes Event into EventQueue
//
//  std::thread  stats_thread_  [SCHED_OTHER, lowest priority]
//    └─ reads /proc/stat, /proc/meminfo, /sys/class/thermal  ~1 Hz
//       writes into SharedData
//
//  AimRT subscribers (called on AimRT channel threads):
//    /joint_states    → SharedData::joints
//    /imu_data        → SharedData::imu_*
//    /joy_vel / joy   → SharedData::joy_*
//
//  Priority rationale (RK3588):
//    Policy inference + IMU streaming + joint control run at RT priority.
//    GPIO polling and SBC stats are best-effort; they never block the
//    AimRT executor or any real-time thread.
// =============================================================================

#include <atomic>
#include <memory>
#include <thread>
#include <vector>

#include "aimrt_module_cpp_interface/module_base.h"

#include "event_queue.h"
#include "shared_data.h"
#include "ui_renderer.h"
#include "oled_driver.hpp"

namespace mybipedal_deploy::oled_module {

class OledModule : public aimrt::ModuleBase {
 public:
  OledModule()           = default;
  ~OledModule() override = default;

  [[nodiscard]] aimrt::ModuleInfo Info() const override {
    return aimrt::ModuleInfo{.name = "OledModule"};
  }

  bool Initialize(aimrt::CoreRef core) override;
  bool Start() override;
  void Shutdown() override;

 private:
  // ── AimRT main loop (runs on executor) ───────────────────────────────────
  bool MainLoop();

  // ── Background thread bodies ──────────────────────────────────────────────
  void GpioThreadFunc();   ///< encoder + buttons → EventQueue
  void StatsThreadFunc();  ///< SBC stats → SharedData

  // ── Helpers ───────────────────────────────────────────────────────────────
  void SetThreadLowestPriority(std::thread& t);

 private:
  // AimRT handles
  aimrt::CoreRef             core_;
  aimrt::executor::ExecutorRef executor_;
  std::vector<aimrt::channel::SubscriberRef> subs_;

  // OLED hardware
  std::unique_ptr<OledDriver> oled_;

  // Shared state
  SharedData  shared_data_;
  EventQueue  event_queue_;
  UIContext   ui_ctx_;

  // Background threads (lowest OS priority)
  std::thread gpio_thread_;
  std::thread stats_thread_;
  std::atomic<bool> run_flag_{true};

  // Config (loaded from YAML)
  std::string i2c_device_;
  uint8_t     oled_addr_{0x3C};
  std::chrono::seconds sleep_timeout_{30};

  // GPIO pin config (loaded from YAML or defaulting to Radxa 5B+ layout)
  std::string gpio_chip_encoder_{"gpiochip3"};
  unsigned    gpio_clk_{13};
  unsigned    gpio_dt_{15};
  unsigned    gpio_sw_{16};
  std::string gpio_chip_back_{"gpiochip4"};
  unsigned    gpio_back_{20};

  // Topic names (loaded from YAML)
  std::string sub_joint_state_topic_;
  std::string sub_imu_topic_;
  std::string sub_joy_topic_;
};

}  // namespace mybipedal_deploy::oled_module
