#pragma once

// ─────────────────────────────────────────────────────────────────────────────
//  oled_module.h — AimRT module điều khiển OLED SH1106
//
//  Thread model (sau khi refactor):
//    ┌─────────────────────────────────────────────────────┐
//    │  AimRT executor "oled_render_thread"                │
//    │  └─ RenderLoop()                                    │
//    │       ├─ display_.tick()      (render 1 frame)      │
//    │       ├─ RefreshSbcStatus()   (mỗi ~1s)             │
//    │       └─ sleep_until()        (giữ fps đúng nhịp)   │
//    ├─────────────────────────────────────────────────────┤
//    │  AimRT subscriber callbacks  (bất kỳ thread nào)   │
//    │  └─ display_.update_*()      (mutex-guarded)        │
//    ├─────────────────────────────────────────────────────┤
//    │  RotaryEncoder::poll_thread_  (libgpiod IRQ wait)   │
//    │  └─ ghi delta_/pressed_ atomic — không làm gì khác  │
//    └─────────────────────────────────────────────────────┘
//
//  Không có thread nào khác.  OledDisplay không tự spawn thread.
// ─────────────────────────────────────────────────────────────────────────────

#include <atomic>
#include <chrono>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "aimrt_module_cpp_interface/module_base.h"
#include "oled_driver.hpp"

using namespace std::chrono;

namespace mybipedal_deploy::oled_module {

class OledModule : public aimrt::ModuleBase {
 public:
  OledModule()  = default;
  ~OledModule() override = default;

  [[nodiscard]] aimrt::ModuleInfo Info() const override {
    return aimrt::ModuleInfo{.name = "OledModule"};
  }

  bool Initialize(aimrt::CoreRef core) override;
  bool Start()                         override;
  void Shutdown()                      override;

 private:
  bool RenderLoop();   // chạy trên AimRT executor

  // SBC status — đọc /proc, /sys từ executor thread
  struct CpuStat { uint64_t idle = 0, total = 0; };
  void                   RefreshSbcStatus();
  std::array<uint8_t, 8> SampleCpuUsage();
  float                  ReadCpuTemp();
  uint8_t                ReadRamPct();
  float                  ReadLoadAvg();

 private:
  // ── AimRT handles ─────────────────────────────────────────────────────────
  aimrt::CoreRef                             core_;
  aimrt::executor::ExecutorRef               executor_;
  std::vector<aimrt::channel::SubscriberRef> subs_;

  // ── OLED driver  (không có internal thread) ───────────────────────────────
  std::unique_ptr<oled::OledDisplay> display_;

  // ── Config ────────────────────────────────────────────────────────────────
  struct Cfg {
    std::string i2c_device  = "/dev/i2c-1";
    uint8_t     i2c_addr    = 0x3C;
    int         fps         = 20;
    int         boot_frames = 60;

    oled::RotaryEncoder::Config encoder;

    std::string sub_joint_state = "/joint_states";
    std::string sub_imu_data    = "/imu/data";
    std::string sub_joy_vel     = "/cmd_vel_limiter";
    std::string sub_battery     = "/battery_pct";

    std::vector<std::pair<std::string /*topic*/, std::string /*label*/>> state_topics;
    std::vector<std::string> joint_list;
  } cfg_;

  // ── Runtime ───────────────────────────────────────────────────────────────
  std::atomic_bool run_flag_{true};

  // joint index map — populated on first JointState message (executor thread)
  std::unordered_map<std::string, int> joint_idx_;

  // SBC subsampling
  std::array<CpuStat, 8> prev_cpu_{};
  bool                   cpu_init_  = false;
  int                    sbc_frame_ = 0;
};

}  // namespace mybipedal_deploy::oled_module