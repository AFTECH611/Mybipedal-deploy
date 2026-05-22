// ─────────────────────────────────────────────────────────────────────────────
//  oled_module.cc
//
//  Initialize()  →  đọc YAML, tạo OledDisplay, đăng ký subscribers
//  Start()       →  post RenderLoop() lên AimRT executor
//  RenderLoop()  →  vòng lặp duy nhất: tick() + SBC refresh + sleep_until()
//  Shutdown()    →  set run_flag_ = false
// ─────────────────────────────────────────────────────────────────────────────

#include "oled_module/oled_module.h"

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>

#include "aimrt_module_ros2_interface/channel/ros2_channel.h"
#include <geometry_msgs/msg/twist.hpp>
#include <yaml-cpp/yaml.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include "oled_module/global.h"


namespace mybipedal_deploy::oled_module {

// ─────────────────────────────────────────────────────────────────────────────
//  Helpers
// ─────────────────────────────────────────────────────────────────────────────

static std::string GetPrimaryIp() {
  struct ifaddrs* list = nullptr;
  if (::getifaddrs(&list) != 0 || !list) return "?.?.?.?";
  std::string result = "?.?.?.?";
  for (auto* p = list; p; p = p->ifa_next) {
    if (!p->ifa_addr || p->ifa_addr->sa_family != AF_INET) continue;
    if (p->ifa_name && std::string(p->ifa_name) == "lo") continue;
    char buf[INET_ADDRSTRLEN]{};
    auto* sin = reinterpret_cast<struct sockaddr_in*>(p->ifa_addr);
    if (::inet_ntop(AF_INET, &sin->sin_addr, buf, sizeof(buf))) {
      result = buf; break;
    }
  }
  ::freeifaddrs(list);
  return result;
}

static std::string ToUpper(std::string s) {
  for (auto& c : s) c = static_cast<char>(std::toupper((unsigned char)c));
  return s;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Initialize
// ─────────────────────────────────────────────────────────────────────────────

bool OledModule::Initialize(aimrt::CoreRef core) {
  core_ = core;
  // ── 1. Đọc YAML ────────────────────────────────────────────────────────────
  auto file_path = core_.GetConfigurator().GetConfigFilePath();
  try {
    if (!file_path.empty()) {
      YAML::Node cfg = YAML::LoadFile(file_path.data());

      if (cfg["oled_fps"])         cfg_.fps         = cfg["oled_fps"].as<int>();
      if (cfg["oled_boot_frames"]) cfg_.boot_frames = cfg["oled_boot_frames"].as<int>();
      if (cfg["oled_i2c_device"])  cfg_.i2c_device  = cfg["oled_i2c_device"].as<std::string>();
      if (cfg["oled_i2c_addr"])    cfg_.i2c_addr    = static_cast<uint8_t>(cfg["oled_i2c_addr"].as<int>());

      if (cfg["oled_encoder"]) {
        auto e = cfg["oled_encoder"];
        if (e["gpiochip"])    cfg_.encoder.gpiochip    = e["gpiochip"].as<std::string>();
        if (e["pin_a"])       cfg_.encoder.pin_a       = e["pin_a"].as<unsigned>();
        if (e["pin_b"])       cfg_.encoder.pin_b       = e["pin_b"].as<unsigned>();
        if (e["pin_btn"])     cfg_.encoder.pin_btn     = e["pin_btn"].as<unsigned>();
        if (e["debounce_ms"]) cfg_.encoder.debounce_ms = e["debounce_ms"].as<int>();
        if (e["invert_dir"])  cfg_.encoder.invert_dir  = e["invert_dir"].as<bool>();
      }

      // Topic names — tái sử dụng keys từ control_module YAML
      if (cfg["sub_joint_state_name"]) cfg_.sub_joint_state = cfg["sub_joint_state_name"].as<std::string>();
      if (cfg["sub_imu_data_name"])    cfg_.sub_imu_data    = cfg["sub_imu_data_name"].as<std::string>();
      if (cfg["sub_joy_vel_name"])     cfg_.sub_joy_vel     = cfg["sub_joy_vel_name"].as<std::string>();
      if (cfg["sub_battery_name"])     cfg_.sub_battery     = cfg["sub_battery_name"].as<std::string>();

      if (cfg["joint_list"])
        cfg_.joint_list = cfg["joint_list"].as<std::vector<std::string>>();

      // robot_states → cặp (trigger_topic, "WALK" / "STAND" / ...)
      if (cfg["robot_states"]) {
        for (auto it = cfg["robot_states"].begin(); it != cfg["robot_states"].end(); ++it) {
          cfg_.state_topics.emplace_back(
            it->second["trigger_topic"].as<std::string>(),
            ToUpper(it->first.as<std::string>())
          );
        }
      }
    }
  } catch (const std::exception& e) {
    AIMRT_ERROR("OledModule config error: {}", e.what());
    return false;
  }

  // ── 2. Tạo OledDisplay ────────────────────────────────────────────────────
  oled::OledDisplay::Config dcfg;
  dcfg.i2c_device  = cfg_.i2c_device;
  dcfg.i2c_addr    = cfg_.i2c_addr;
  dcfg.fps         = cfg_.fps;
  dcfg.boot_frames = cfg_.boot_frames;
  dcfg.encoder     = cfg_.encoder;
  try {
    display_ = std::make_unique<oled::OledDisplay>(dcfg);
  } catch (const std::exception& e) {
    AIMRT_ERROR("OledDisplay init failed: {}", e.what());
    return false;
  }

  // Khởi tạo joint index map với sentinel -1
  for (const auto& j : cfg_.joint_list) joint_idx_[j] = -1;

  // ── 3. Subscribers ────────────────────────────────────────────────────────

  // 3a. JointState
  subs_.push_back(core_.GetChannelHandle().GetSubscriber(cfg_.sub_joint_state));
  {
    bool ret = aimrt::channel::Subscribe<sensor_msgs::msg::JointState>(
      subs_.back(),
      [this](const std::shared_ptr<const sensor_msgs::msg::JointState>& msg) {
        // Xây index map một lần duy nhất
        if (!joint_idx_.empty() && joint_idx_.begin()->second == -1) {
          for (size_t i = 0; i < msg->name.size(); ++i)
            if (joint_idx_.count(msg->name[i]))
              joint_idx_[msg->name[i]] = static_cast<int>(i);
        }

        std::vector<oled::JointState> js;
        js.reserve(cfg_.joint_list.size());
        for (const auto& name : cfg_.joint_list) {
          auto it = joint_idx_.find(name);
          if (it == joint_idx_.end() || it->second < 0) continue;
          int idx = it->second;
          oled::JointState s;
          s.name = name;
          if (idx < (int)msg->position.size()) s.pos    = (float)msg->position[idx];
          if (idx < (int)msg->velocity.size()) s.vel    = (float)msg->velocity[idx];
          if (idx < (int)msg->effort.size())   s.torque = (float)msg->effort[idx];
          js.push_back(s);
        }
        display_->update_joints(js);
      });
    AIMRT_CHECK_ERROR_THROW(ret, "OledModule: subscribe joint_states failed");
  }

  // 3b. IMU
  subs_.push_back(core_.GetChannelHandle().GetSubscriber(cfg_.sub_imu_data));
  {
    bool ret = aimrt::channel::Subscribe<sensor_msgs::msg::Imu>(
      subs_.back(),
      [this](const std::shared_ptr<const sensor_msgs::msg::Imu>& msg) {
        oled::ImuState imu;
        imu.ax = (float)msg->linear_acceleration.x;
        imu.ay = (float)msg->linear_acceleration.y;
        imu.az = (float)msg->linear_acceleration.z;
        imu.gx = (float)msg->angular_velocity.x;
        imu.gy = (float)msg->angular_velocity.y;
        imu.gz = (float)msg->angular_velocity.z;
        // Tính roll/pitch từ accelerometer
        imu.roll  = std::atan2(imu.ay, imu.az);
        imu.pitch = std::atan2(-imu.ax, std::sqrt(imu.ay*imu.ay + imu.az*imu.az));
        display_->update_imu(imu);
      });
    AIMRT_CHECK_ERROR_THROW(ret, "OledModule: subscribe imu failed");
  }

  // 3c. cmd_vel → JoyState (linear.x=forward, angular.z=yaw)
  subs_.push_back(core_.GetChannelHandle().GetSubscriber(cfg_.sub_joy_vel));
  {
    bool ret = aimrt::channel::Subscribe<geometry_msgs::msg::Twist>(
      subs_.back(),
      [this](const std::shared_ptr<const geometry_msgs::msg::Twist>& msg) {
        oled::JoyState joy;
        joy.ly = (float)msg->linear.x;   // forward
        joy.lx = (float)msg->linear.y;   // lateral
        joy.rx = (float)msg->angular.z;  // yaw
        display_->update_joy(joy);
      });
    AIMRT_CHECK_ERROR_THROW(ret, "OledModule: subscribe cmd_vel failed");
  }

  // 3d. Battery  (std_msgs/Float32, giá trị 0–100)
  subs_.push_back(core_.GetChannelHandle().GetSubscriber(cfg_.sub_battery));
  {
    bool ret = aimrt::channel::Subscribe<std_msgs::msg::Float32>(
      subs_.back(),
      [this](const std::shared_ptr<const std_msgs::msg::Float32>& msg) {
        display_->update_battery(
          (uint8_t)std::clamp((int)msg->data, 0, 100));
      });
    AIMRT_CHECK_ERROR_THROW(ret, "OledModule: subscribe battery failed");
  }

  // 3e. State trigger topics  →  update mode label + push log
  {
    std::set<std::string> seen;
    for (const auto& [topic, label] : cfg_.state_topics) {
      if (!seen.insert(topic).second) continue;
      subs_.push_back(core_.GetChannelHandle().GetSubscriber(topic));
      bool ret = aimrt::channel::Subscribe<std_msgs::msg::Float32>(
        subs_.back(),
        [this, label](const std::shared_ptr<const std_msgs::msg::Float32>&) {
          display_->update_mode(label);
          display_->push_log("Mode -> " + label);
        });
      AIMRT_CHECK_ERROR_THROW(ret, "OledModule: subscribe state topic failed");
    }
  }

  // ── 4. Executor ───────────────────────────────────────────────────────────
  executor_ = core_.GetExecutorManager().GetExecutor("oled_render_thread");
  AIMRT_CHECK_ERROR_THROW(executor_,
    "OledModule: executor 'oled_render_thread' not found. "
    "Add it to your AimRT app config.");

  AIMRT_INFO("OledModule initialized (i2c={}, fps={}, boot_frames={}).",
             cfg_.i2c_device, cfg_.fps, cfg_.boot_frames);
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Start
// ─────────────────────────────────────────────────────────────────────────────

bool OledModule::Start() {
  try {
    display_->update_ip(GetPrimaryIp());
    executor_.Execute([this]() { RenderLoop(); });
    AIMRT_INFO("OledModule started.");
  } catch (const std::exception& e) {
    AIMRT_ERROR("OledModule start failed: {}", e.what());
    return false;
  }
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Shutdown
// ─────────────────────────────────────────────────────────────────────────────

void OledModule::Shutdown() {
  run_flag_.store(false);
}

// ─────────────────────────────────────────────────────────────────────────────
//  RenderLoop  —  chạy hoàn toàn trên AimRT executor "oled_render_thread"
//
//  Cấu trúc một iteration:
//    ┌────────────────────────────────────────────┐
//    │ sleep_until(next_frame)                    │  ← giữ fps chính xác
//    │ display_.tick()                            │  ← encoder FSM + render
//    │ if (sbc_frame_ >= fps) RefreshSbcStatus()  │  ← mỗi ~1 giây
//    └────────────────────────────────────────────┘
//
//  Tại sao sleep_until trước tick() chứ không phải sau?
//    → Render + I2C flush mất ~2–5ms.  Nếu sleep sau thì mỗi frame bị trễ
//      thêm thời gian render.  Sleep trước đảm bảo period tuyệt đối.
// ─────────────────────────────────────────────────────────────────────────────

bool OledModule::RenderLoop() {
  try {
    AIMRT_INFO("OledModule RenderLoop started (fps={}).", cfg_.fps);

    // Khởi tạo phần cứng từ executor thread (I2C init phải đúng thread)
    display_->init();

    const auto period = nanoseconds(1'000'000'000LL / cfg_.fps);
    auto next_frame   = high_resolution_clock::now() + period;

    while (run_flag_.load(std::memory_order_relaxed)) {
      // Ngủ đúng đến đầu frame tiếp theo
      std::this_thread::sleep_until(next_frame);
      next_frame += period;

      if (!run_flag_.load()) break;

      // Render một frame (encoder FSM + draw + I2C flush)
      if (!display_->tick()) {
        AIMRT_ERROR("OledModule: display tick() failed (I2C error?). Stopping.");
        break;
      }

      // Refresh SBC status mỗi fps frames (~1 giây)
      // Đặt SAU tick() để không ảnh hưởng đến jitter của frame đầu tiên
      if (++sbc_frame_ >= cfg_.fps) {
        sbc_frame_ = 0;
        RefreshSbcStatus();   // đọc /proc, /sys — ~1ms, chấp nhận được
      }
    }

    display_->power_off();
    AIMRT_INFO("OledModule RenderLoop exited.");
  } catch (const std::exception& e) {
    AIMRT_ERROR("OledModule RenderLoop exception: {}", e.what());
    return false;
  }
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  SBC status helpers  (gọi từ executor thread, không cần mutex)
// ─────────────────────────────────────────────────────────────────────────────

void OledModule::RefreshSbcStatus() {
  oled::SbcStatus s;
  s.cpu_pct  = SampleCpuUsage();
  s.cpu_temp = ReadCpuTemp();
  s.ram_pct  = ReadRamPct();
  s.load_1m  = ReadLoadAvg();
  display_->update_sbc(s);   // mutex bên trong update_sbc()
}

std::array<uint8_t, 8> OledModule::SampleCpuUsage() {
  std::ifstream f("/proc/stat");
  std::array<uint8_t, 8> result{};
  if (!f) return result;
  std::string line;
  int core = 0;
  while (std::getline(f, line) && core < 8) {
    if (line.rfind("cpu", 0) != 0 || line.size() < 4 ||
        !std::isdigit((unsigned char)line[3])) continue;
    std::istringstream ss(line);
    std::string tag;
    uint64_t user, nice, sys, idle, iowait=0, irq=0, softirq=0;
    ss >> tag >> user >> nice >> sys >> idle >> iowait >> irq >> softirq;
    uint64_t total  = user+nice+sys+idle+iowait+irq+softirq;
    uint64_t idle_d = idle  - prev_cpu_[core].idle;
    uint64_t tot_d  = total - prev_cpu_[core].total;
    if (cpu_init_ && tot_d > 0)
      result[core] = (uint8_t)(100*(tot_d - idle_d)/tot_d);
    prev_cpu_[core] = {idle, total};
    ++core;
  }
  cpu_init_ = true;
  return result;
}

float OledModule::ReadCpuTemp() {
  for (int z = 0; z < 6; ++z) {
    char path[64];
    std::snprintf(path, sizeof(path), "/sys/class/thermal/thermal_zone%d/temp", z);
    std::ifstream f(path);
    if (!f) continue;
    int milli = 0; f >> milli;
    if (milli > 0) return milli / 1000.f;
  }
  return 0.f;
}

uint8_t OledModule::ReadRamPct() {
  std::ifstream f("/proc/meminfo");
  if (!f) return 0;
  uint64_t total=0, avail=0;
  std::string line;
  while (std::getline(f, line)) {
    if (line.rfind("MemTotal:", 0) == 0)
      std::sscanf(line.c_str(), "MemTotal: %lu kB", &total);
    else if (line.rfind("MemAvailable:", 0) == 0)
      std::sscanf(line.c_str(), "MemAvailable: %lu kB", &avail);
  }
  return total ? (uint8_t)(100*(total-avail)/total) : 0;
}

float OledModule::ReadLoadAvg() {
  std::ifstream f("/proc/loadavg");
  float l1 = 0.f;
  if (f) f >> l1;
  return l1;
}

}  // namespace mybipedal_deploy::oled_module