// =============================================================================
//  oled_module.cc  –  AimRT module: SSD1306 OLED display driver
//
//  Thread model (RK3588 priority rationale):
//    ┌─ AimRT executor "oled_pub_thread"  ─────────────── inherited priority ─┐
//    │   MainLoop()  – UI event loop (200 ms tick)                             │
//    │   AimRT subscriber callbacks – fill SharedData                         │
//    └────────────────────────────────────────────────────────────────────────┘
//    ┌─ std::thread  gpio_thread_   [SCHED_OTHER, nice +19] ──── lowest ───────┐
//    │   Polls encoder + buttons at 2 kHz via libgpiod                         │
//    └────────────────────────────────────────────────────────────────────────┘
//    ┌─ std::thread  stats_thread_  [SCHED_OTHER, nice +19] ──── lowest ───────┐
//    │   Reads /proc/stat, /proc/meminfo, /sys thermal  ~1 Hz                  │
//    └────────────────────────────────────────────────────────────────────────┘
//
//  The two std::threads are deliberately pinned to the lowest OS scheduling
//  priority (nice +19) so they never compete with policy inference, IMU
//  streaming, or joint control tasks.
// =============================================================================

#include "oled_module/oled_module.h"

#include <arpa/inet.h>
#include <pthread.h>
#include <sys/resource.h>

#include <chrono>
#include <format>
#include <stdexcept>

#include "aimrt_module_ros2_interface/channel/ros2_channel.h"
#include "geometry_msgs/msg/twist.hpp"
#include "my_ros2_proto/msg/joy_stick_data.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "oled_module/gpio_input.h"
#include "oled_module/sbc_stats.h"
#include "oled_module/ui_renderer.h"

using namespace std::chrono_literals;

namespace mybipedal_deploy::oled_module {

// ============================================================================
//  Initialize
// ============================================================================
bool OledModule::Initialize(aimrt::CoreRef core) {
  core_ = core;
  subs_.clear();

  auto file_path = core_.GetConfigurator().GetConfigFilePath();
  try {
    YAML::Node cfg = YAML::LoadFile(file_path.data());

    // ── Hardware config ───────────────────────────────────────────────────
    i2c_device_  = cfg["i2c_device"].as<std::string>("/dev/i2c-7");
    oled_addr_   = static_cast<uint8_t>(cfg["oled_addr"].as<int>(0x3C));
    sleep_timeout_ = std::chrono::seconds(cfg["sleep_timeout_sec"].as<int>(30));

    // ── GPIO pin config ───────────────────────────────────────────────────
    if (cfg["gpio"]) {
      auto g = cfg["gpio"];
      gpio_chip_encoder_ = g["chip_encoder"].as<std::string>("gpiochip3");
      gpio_clk_          = g["clk"].as<unsigned>(13);
      gpio_dt_           = g["dt"].as<unsigned>(15);
      gpio_sw_           = g["sw"].as<unsigned>(16);
      gpio_chip_back_    = g["chip_back"].as<std::string>("gpiochip4");
      gpio_back_         = g["back"].as<unsigned>(20);
    }

    // ── Topic names ───────────────────────────────────────────────────────
    sub_joint_state_topic_ = cfg["sub_joint_state_name"].as<std::string>("/joint_states");
    sub_imu_topic_         = cfg["sub_imu_data_name"].as<std::string>("/imu_data");
    sub_joy_topic_         = cfg["sub_joy_name"].as<std::string>("/joy_vel");

    // ── OLED hardware init ────────────────────────────────────────────────
    auto i2c = std::make_unique<I2CDevice>(i2c_device_, oled_addr_);
    if (!i2c->open())
      AIMRT_ERROR_THROW("Cannot open I2C device {}", i2c_device_);

    oled_ = std::make_unique<OledDriver>(std::move(i2c));
    if (!oled_->init())
      AIMRT_ERROR_THROW("SSD1306 init sequence failed");

    oled_->clear();
    oled_->display();

    // ── Initial shared data ───────────────────────────────────────────────
    shared_data_.robot_ip   = getLocalIP();
    shared_data_.robot_name = cfg["robot_name"].as<std::string>("MyBipedal");

    // ── Subscribe: joint states ───────────────────────────────────────────
    subs_.push_back(core_.GetChannelHandle().GetSubscriber(sub_joint_state_topic_));
    bool ret = aimrt::channel::Subscribe<sensor_msgs::msg::JointState>(
        subs_.back(),
        [this](const std::shared_ptr<const sensor_msgs::msg::JointState>& msg) {
          std::lock_guard lock(shared_data_.mtx);
          // Rebuild joint list to match incoming message order
          shared_data_.joints.resize(msg->name.size());
          for (size_t i = 0; i < msg->name.size(); ++i) {
            shared_data_.joints[i].name = msg->name[i];
            shared_data_.joints[i].pos  =
                i < msg->position.size() ? static_cast<float>(msg->position[i]) : 0.f;
            // Temperature not in JointState; leave at 0 or fill from separate topic
            // shared_data_.joints[i].temp = ...
          }
        });
    AIMRT_CHECK_ERROR_THROW(ret, "Subscribe {} failed.", sub_joint_state_topic_);

    // ── Subscribe: IMU ────────────────────────────────────────────────────
    subs_.push_back(core_.GetChannelHandle().GetSubscriber(sub_imu_topic_));
    ret = aimrt::channel::Subscribe<sensor_msgs::msg::Imu>(
        subs_.back(),
        [this](const std::shared_ptr<const sensor_msgs::msg::Imu>& msg) {
          std::lock_guard lock(shared_data_.mtx);
          shared_data_.imu_gx = static_cast<float>(msg->angular_velocity.x);
          shared_data_.imu_gy = static_cast<float>(msg->angular_velocity.y);
          shared_data_.imu_gz = static_cast<float>(msg->angular_velocity.z);
          shared_data_.imu_ax = static_cast<float>(msg->linear_acceleration.x);
          shared_data_.imu_ay = static_cast<float>(msg->linear_acceleration.y);
          shared_data_.imu_az = static_cast<float>(msg->linear_acceleration.z);
        });
    AIMRT_CHECK_ERROR_THROW(ret, "Subscribe {} failed.", sub_imu_topic_);

    // ── Subscribe: Joystick ───────────────────────────────────────────────
    // Supports both geometry_msgs/Twist (velocity command) and
    // my_ros2_proto/JoyStickData (raw joystick). Topic name drives choice.
    subs_.push_back(core_.GetChannelHandle().GetSubscriber(sub_joy_topic_));
    ret = aimrt::channel::Subscribe<geometry_msgs::msg::Twist>(
        subs_.back(),
        [this](const std::shared_ptr<const geometry_msgs::msg::Twist>& msg) {
          std::lock_guard lock(shared_data_.mtx);
          // Map Twist to joystick axes for display
          shared_data_.joy_lx = static_cast<float>(msg->linear.y);
          shared_data_.joy_ly = static_cast<float>(msg->linear.x);
          shared_data_.joy_rx = static_cast<float>(msg->angular.z);
          shared_data_.joy_ry = 0.f;
        });
    AIMRT_CHECK_ERROR_THROW(ret, "Subscribe {} failed.", sub_joy_topic_);

    // ── Executor ──────────────────────────────────────────────────────────
    executor_ = core_.GetExecutorManager().GetExecutor("oled_pub_thread");
    AIMRT_CHECK_ERROR_THROW(executor_, "Cannot get executor 'oled_pub_thread'.");

  } catch (const std::exception& e) {
    AIMRT_ERROR("OledModule::Initialize failed: {}", e.what());
    return false;
  }

  AIMRT_INFO("OledModule::Initialize succeeded.");
  return true;
}

// ============================================================================
//  Start
// ============================================================================
bool OledModule::Start() {
  try {
    // UI state: start in sleep
    ui_ctx_.state = UIState::SLEEPING;
    oled_->setSleep(true);

    // Launch GPIO thread (lowest priority)
    gpio_thread_ = std::thread(&OledModule::GpioThreadFunc, this);
    SetThreadLowestPriority(gpio_thread_);

    // Launch SBC stats thread (lowest priority)
    stats_thread_ = std::thread(&OledModule::StatsThreadFunc, this);
    SetThreadLowestPriority(stats_thread_);

    // Launch main UI loop on AimRT executor
    executor_.Execute([this]() { MainLoop(); });

    AIMRT_INFO("OledModule::Start succeeded.");
  } catch (const std::exception& e) {
    AIMRT_ERROR("OledModule::Start failed: {}", e.what());
    return false;
  }
  return true;
}

// ============================================================================
//  Shutdown
// ============================================================================
void OledModule::Shutdown() {
  run_flag_.store(false);

  // Wake the event queue so MainLoop can exit cleanly
  event_queue_.push(Event::BTN_BACK);

  if (gpio_thread_.joinable())  gpio_thread_.join();
  if (stats_thread_.joinable()) stats_thread_.join();

  // Blank display on shutdown
  if (oled_) {
    oled_->clear();
    oled_->display();
    oled_->setSleep(true);
  }
}

// ============================================================================
//  MainLoop  –  runs on AimRT executor "oled_pub_thread"
// ============================================================================
bool OledModule::MainLoop() {
  AIMRT_INFO("OledModule::MainLoop started.");
  try {
    while (run_flag_) {
      // Block up to 200 ms for an input event
      auto ev_opt = event_queue_.pop(200ms);

      if (!run_flag_) break;

      if (ev_opt) {
        bool was_sleeping = (ui_ctx_.state == UIState::SLEEPING);
        handleEvent(*ev_opt, ui_ctx_, *oled_);
        if (was_sleeping) {
          // Swallow mechanical bounce events emitted during wake-up
          event_queue_.clear();
        }
      }

      // Auto-sleep
      if (ui_ctx_.state != UIState::SLEEPING &&
          ui_ctx_.isTimedOut(sleep_timeout_)) {
        AIMRT_INFO("OledModule: inactivity timeout – going to sleep.");
        ui_ctx_.state = UIState::SLEEPING;
        oled_->setSleep(true);
        oled_->clear();
        oled_->display();
        continue;
      }

      // Render
      if (ui_ctx_.state != UIState::SLEEPING) {
        DataSnapshot snap = shared_data_.snapshot();
        renderUI(*oled_, ui_ctx_, snap);
      }
    }
    AIMRT_INFO("OledModule::MainLoop exited.");
  } catch (const std::exception& e) {
    AIMRT_ERROR("OledModule::MainLoop exception: {}", e.what());
    return false;
  }
  return true;
}

// ============================================================================
//  GpioThreadFunc  –  encoder + button polling (std::thread, lowest priority)
// ============================================================================
void OledModule::GpioThreadFunc() {
  AIMRT_INFO("OledModule::GpioThreadFunc started.");
  try {
    const std::string enc_chip_path  = "/dev/" + gpio_chip_encoder_;
    const std::string back_chip_path = "/dev/" + gpio_chip_back_;

    InputPin clk ({enc_chip_path.c_str(),  gpio_clk_});
    InputPin dt  ({enc_chip_path.c_str(),  gpio_dt_});
    InputPin sw  ({enc_chip_path.c_str(),  gpio_sw_});
    InputPin back({back_chip_path.c_str(), gpio_back_});

    int lastClk  = clk.Read();
    int lastSw   = sw.Read();
    int lastBack = back.Read();

    while (run_flag_) {
      // ── Encoder rotation (falling edge on CLK) ─────────────────────────
      int currentClk = clk.Read();
      if (currentClk != lastClk && currentClk == 0) {
        int dtState = dt.Read();
        event_queue_.push(dtState != currentClk ? Event::ENC_CCW : Event::ENC_CW);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      lastClk = currentClk;

      // ── Encoder push (debounced) ────────────────────────────────────────
      int swState = sw.Read();
      if (swState != lastSw) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        int confirmed = sw.Read();
        if (confirmed == swState) {
          if (confirmed == 0) event_queue_.push(Event::ENC_PUSH);
          lastSw = confirmed;
        }
      }

      // ── Back button (debounced) ────────────────────────────────────────
      int backState = back.Read();
      if (backState != lastBack) {
        if (backState == 0) {
          event_queue_.push(Event::BTN_BACK);
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        lastBack = backState;
      }

      // 2 kHz poll cycle
      std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
  } catch (const std::exception& e) {
    AIMRT_ERROR("OledModule::GpioThreadFunc fatal: {}", e.what());
    run_flag_.store(false);
  }
  AIMRT_INFO("OledModule::GpioThreadFunc stopped.");
}

// ============================================================================
//  StatsThreadFunc  –  SBC stats update ~1 Hz (std::thread, lowest priority)
// ============================================================================
void OledModule::StatsThreadFunc() {
  AIMRT_INFO("OledModule::StatsThreadFunc started.");
  while (run_flag_.load(std::memory_order_relaxed)) {
    auto cores = readCpuUsage();  // blocks ~200 ms internally
    float temp = readCpuTemp();
    uint64_t total_kb, avail_kb;
    readRamInfo(total_kb, avail_kb);

    // Update IP every cycle (cheap and handles DHCP lease changes)
    std::string ip = getLocalIP();

    {
      std::lock_guard lock(shared_data_.mtx);
      shared_data_.cpu_core_pct = std::move(cores);
      shared_data_.cpu_temp_c   = temp;
      shared_data_.ram_total_kb = total_kb;
      shared_data_.ram_avail_kb = avail_kb;
      shared_data_.robot_ip     = std::move(ip);
    }

    // Sleep the remainder of the ~1 s interval
    std::this_thread::sleep_for(800ms);
  }
  AIMRT_INFO("OledModule::StatsThreadFunc stopped.");
}

// ============================================================================
//  SetThreadLowestPriority  –  nice +19 via setpriority(PRIO_PROCESS)
//
//  We use SCHED_OTHER + nice +19 rather than sched_setscheduler so we don't
//  need CAP_SYS_NICE.  This is sufficient to ensure these threads yield to
//  any SCHED_FIFO/SCHED_RR real-time task (policy inference, joint control).
// ============================================================================
void OledModule::SetThreadLowestPriority(std::thread& t) {
  // pthread_setschedparam with SCHED_OTHER + max nice
  sched_param param{};
  param.sched_priority = 0;  // must be 0 for SCHED_OTHER
  int rc = ::pthread_setschedparam(t.native_handle(), SCHED_OTHER, &param);
  if (rc != 0) {
    AIMRT_ERROR("OledModule: pthread_setschedparam failed ({}), trying setpriority.", rc);
  }
  // Also renice to +19 for maximum yielding behaviour
  ::setpriority(PRIO_PROCESS, 0, 19);
}

}  // namespace mybipedal_deploy::oled_module
