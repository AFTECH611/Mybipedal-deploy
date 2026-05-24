#pragma once

// =============================================================================
//  shared_data.h  –  Thread-safe live data shared between AimRT subscribers
//                    and the OLED UI render loop.
//
//  Design:
//    - SharedData  : mutex-protected, written by subscriber callbacks
//    - DataSnapshot: plain-old-data copy produced by SharedData::snapshot()
//                    for lock-free reading in the render thread
// =============================================================================

#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

namespace mybipedal_deploy::oled_module {

// ---------------------------------------------------------------------------
//  Per-joint display info
// ---------------------------------------------------------------------------
struct JointInfo {
  std::string name;
  float       pos{0.f};
  float       temp{0.f};
};

// ---------------------------------------------------------------------------
//  DataSnapshot  –  plain-old-data, no mutex, safe to read from render thread
// ---------------------------------------------------------------------------
struct DataSnapshot {
  std::string            robot_ip;
  std::string            robot_name;
  std::vector<JointInfo> joints;

  // IMU (rad/s and m/s²)
  float imu_gx{0}, imu_gy{0}, imu_gz{0};
  float imu_ax{0}, imu_ay{0}, imu_az{0};

  // Joystick axes [-1, 1] and button bitmap
  float    joy_lx{0}, joy_ly{0};
  float    joy_rx{0}, joy_ry{0};
  uint16_t joy_buttons{0};

  // AimRT log ring-buffer
  std::deque<std::string> log_lines;

  // SBC stats (written by statsThread)
  std::vector<float> cpu_core_pct;
  float              cpu_temp_c{0};
  uint64_t           ram_total_kb{0};
  uint64_t           ram_avail_kb{0};
};

// ---------------------------------------------------------------------------
//  SharedData  –  the single live-data store, written from multiple threads
// ---------------------------------------------------------------------------
struct SharedData {
  mutable std::mutex mtx;

  std::string            robot_ip   = "0.0.0.0";
  std::string            robot_name = "MyBipedal";
  std::vector<JointInfo> joints;

  float imu_gx{0}, imu_gy{0}, imu_gz{0};
  float imu_ax{0}, imu_ay{0}, imu_az{0};

  float    joy_lx{0}, joy_ly{0};
  float    joy_rx{0}, joy_ry{0};
  uint16_t joy_buttons{0};

  std::deque<std::string> log_lines;

  std::vector<float> cpu_core_pct;
  float              cpu_temp_c{0};
  uint64_t           ram_total_kb{0};
  uint64_t           ram_avail_kb{0};

  // ── Helpers ───────────────────────────────────────────────────────────────

  /// Append a log line (max 64 entries, drops oldest).
  void pushLog(std::string line) {
    std::lock_guard lock(mtx);
    if (log_lines.size() >= 64) log_lines.pop_front();
    log_lines.push_back(std::move(line));
  }

  /// Thread-safe atomic snapshot for the render loop.
  [[nodiscard]] DataSnapshot snapshot() const {
    std::lock_guard lock(mtx);
    DataSnapshot s;
    s.robot_ip      = robot_ip;
    s.robot_name    = robot_name;
    s.joints        = joints;
    s.imu_gx = imu_gx; s.imu_gy = imu_gy; s.imu_gz = imu_gz;
    s.imu_ax = imu_ax; s.imu_ay = imu_ay; s.imu_az = imu_az;
    s.joy_lx = joy_lx; s.joy_ly = joy_ly;
    s.joy_rx = joy_rx; s.joy_ry = joy_ry;
    s.joy_buttons   = joy_buttons;
    s.log_lines     = log_lines;
    s.cpu_core_pct  = cpu_core_pct;
    s.cpu_temp_c    = cpu_temp_c;
    s.ram_total_kb  = ram_total_kb;
    s.ram_avail_kb  = ram_avail_kb;
    return s;
  }
};

}  // namespace mybipedal_deploy::oled_module
