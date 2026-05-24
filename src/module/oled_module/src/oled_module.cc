// =============================================================================
//  control_module.cc  –  OledUiModule
//
//  Module AimRT điều khiển màn hình OLED SSD1306 cho robot MyBipedal.
//  Nhận dữ liệu qua ba subscriber:
//    • /imu/data          → sensor_msgs::msg::Imu
//    • /joint_states      → sensor_msgs::msg::JointState
//    • /joy               → sensor_msgs::msg::Joy
//
//  Main loop chạy trên AimRT executor (không dùng std::thread riêng cho UI).
//  Luồng GPIO (encoder + nút bấm) và luồng SBC stats vẫn là std::thread
//  riêng vì chúng cần polling phần cứng hoặc sleep dài.
// =============================================================================

#include "control_module/control_module.h"
#include "control_module/global.h"

#include "aimrt_module_ros2_interface/channel/ros2_channel.h"

// OLED driver (giữ nguyên từ main.cpp gốc)
#include "oled_driver.hpp"
#include "i2c.h"

// ── std ──────────────────────────────────────────────────────────────────────
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <format>
#include <fstream>
#include <sstream>
#include <thread>

// ── POSIX / Linux ─────────────────────────────────────────────────────────────
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <gpiod.h>

using namespace std::chrono_literals;
using namespace mybipedal_deploy::oled_ui_module;

// ============================================================================
//  Hằng số phần cứng
// ============================================================================
static constexpr const char* kI2CDev   = "/dev/i2c-7";
static constexpr uint8_t     kOledAddr = 0x3C;
static constexpr auto        kSleepTimeout = 30s;

// ── GPIO pin definitions ──────────────────────────────────────────────────────
struct GpioPin { const char* chip; unsigned int line; };
inline constexpr GpioPin kPinClk  { "/dev/gpiochip3", 13 };
inline constexpr GpioPin kPinDt   { "/dev/gpiochip3", 15 };
inline constexpr GpioPin kPinSw   { "/dev/gpiochip3", 16 };
inline constexpr GpioPin kPinBack { "/dev/gpiochip4", 20 };

// ============================================================================
//  Lớp bọc GPIO (libgpiod)
// ============================================================================
class InputPin {
public:
  explicit InputPin(const GpioPin& pin) {
    chip_ = gpiod_chip_open(pin.chip);
    if (!chip_) throw std::runtime_error("Failed to open chip");
    line_ = gpiod_chip_get_line(chip_, pin.line);
    if (!line_) throw std::runtime_error("Failed to get line");
    gpiod_line_request_config cfg{};
    cfg.consumer     = "oled_ui";
    cfg.request_type = GPIOD_LINE_REQUEST_DIRECTION_INPUT;
    cfg.flags        = GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP;
    if (gpiod_line_request(line_, &cfg, 0) < 0)
      throw std::runtime_error("Failed to request input with pull-up");
  }
  ~InputPin() {
    if (line_) gpiod_line_release(line_);
    if (chip_) gpiod_chip_close(chip_);
  }
  int Read() const { return gpiod_line_get_value(line_); }
private:
  gpiod_chip* chip_{nullptr};
  gpiod_line* line_{nullptr};
};

// ============================================================================
//  Event system (dùng nội bộ trong module)
// ============================================================================
enum class Event { ENC_CW, ENC_CCW, ENC_PUSH, BTN_BACK };

class EventQueue {
public:
  void push(Event e) {
    { std::lock_guard lock(mtx_); q_.push(e); }
    cv_.notify_one();
  }
  std::optional<Event> pop(std::chrono::milliseconds timeout) {
    std::unique_lock lock(mtx_);
    if (!cv_.wait_for(lock, timeout, [&]{ return !q_.empty(); }))
      return std::nullopt;
    Event e = q_.front(); q_.pop(); return e;
  }
  void clear() {
    std::lock_guard lock(mtx_);
    std::queue<Event> empty; std::swap(q_, empty);
  }
private:
  std::queue<Event>       q_;
  std::mutex              mtx_;
  std::condition_variable cv_;
};

// ============================================================================
//  UI State Machine types
// ============================================================================
enum class UIState { SLEEPING, MAIN, MENU, SCREEN };

static constexpr std::array<const char*, 5> kMenuItems = {
  "1 Joint State", "2 IMU State", "3 Joystick", "4 AimRT Log", "5 SBC Status",
};

struct UIContext {
  UIState state         = UIState::SLEEPING;
  int     menu_sel      = 0;
  int     screen_idx    = 0;
  int     scroll_offset = 0;
  std::chrono::steady_clock::time_point last_active = std::chrono::steady_clock::now();

  void touch() { last_active = std::chrono::steady_clock::now(); }
  [[nodiscard]] bool isTimedOut() const {
    return std::chrono::steady_clock::now() - last_active >= kSleepTimeout;
  }
};

// ============================================================================
//  Tiện ích hệ thống
// ============================================================================
static std::string getLocalIP() {
  ifaddrs* ifa = nullptr;
  if (::getifaddrs(&ifa) < 0) return "?.?.?.?";
  std::string result = "0.0.0.0";
  for (auto* p = ifa; p; p = p->ifa_next) {
    if (!p->ifa_addr || p->ifa_addr->sa_family != AF_INET) continue;
    if (std::string(p->ifa_name) == "lo") continue;
    char buf[INET_ADDRSTRLEN];
    auto* sa = reinterpret_cast<sockaddr_in*>(p->ifa_addr);
    ::inet_ntop(AF_INET, &sa->sin_addr, buf, sizeof(buf));
    result = buf; break;
  }
  ::freeifaddrs(ifa);
  return result;
}

static std::vector<float> readCpuUsage() {
  auto readStats = []() {
    std::ifstream f("/proc/stat");
    std::vector<std::array<uint64_t,2>> v;
    std::string line;
    while (std::getline(f, line)) {
      if (line.rfind("cpu", 0) != 0 || line[3] == ' ') continue;
      std::istringstream ss(line.substr(5));
      uint64_t user, nice, sys, idle, iowait, irq, softirq;
      ss >> user >> nice >> sys >> idle >> iowait >> irq >> softirq;
      v.push_back({user+nice+sys+idle+iowait+irq+softirq, idle+iowait});
    }
    return v;
  };
  auto s1 = readStats();
  std::this_thread::sleep_for(200ms);
  auto s2 = readStats();
  std::vector<float> pct;
  for (size_t i = 0; i < std::min(s1.size(), s2.size()); ++i) {
    uint64_t dt = s2[i][0]-s1[i][0], di = s2[i][1]-s1[i][1];
    pct.push_back(dt == 0 ? 0.f : 100.f*(1.f - (float)di/dt));
  }
  return pct;
}

static float readCpuTemp() {
  for (int z = 0; z < 4; ++z) {
    std::ifstream f(std::format("/sys/class/thermal/thermal_zone{}/temp", z));
    if (!f) continue;
    int md; f >> md;
    if (md > 0) return md / 1000.f;
  }
  return 0.f;
}

static void readRamInfo(uint64_t& total_kb, uint64_t& avail_kb) {
  std::ifstream f("/proc/meminfo");
  std::string key, unit; uint64_t val;
  total_kb = avail_kb = 0;
  while (f >> key >> val >> unit) {
    if (key == "MemTotal:")     total_kb = val;
    if (key == "MemAvailable:") avail_kb = val;
  }
}

// ============================================================================
//  Screen renderers  (copy từ main.cpp gốc, không thay đổi logic)
// ============================================================================
static void drawTitle(OledDriver& d, std::string_view title) {
  d.fillRect(0, 0, 128, 10, true);
  int tx = (128 - d.textWidth(title)) / 2;
  d.drawString(tx, 1, title, false);
  d.drawHLine(0, 10, 128, true);
}

static void drawScrollbar(OledDriver& d, int scroll, int content_h) {
  constexpr int VIEW_H = 54, TRACK_Y = 10;
  if (content_h <= VIEW_H) return;
  int max_s = content_h - VIEW_H;
  int bar_h = std::max(6, VIEW_H * VIEW_H / content_h);
  int bar_y = TRACK_Y + scroll * (VIEW_H - bar_h) / max_s;
  d.fillRect(126, bar_y, 2, bar_h, true);
}

static void renderMain(OledDriver& d, const DataSnapshot& sd) {
  d.drawString((128 - d.textWidth("MyBipedal", 2)) / 2, 4,  "MyBipedal", true, 2);
  d.drawString((128 - d.textWidth("By LongVu"))    / 2, 24, "By LongVu", true, 1);
  d.drawHLine(0, 35, 128, true);
  d.drawString(0, 38, "IP:", true);
  d.drawString(18, 38, sd.robot_ip, true);
  d.drawString(0, 54, "  Rotate enc -> Menu  ", true);
}

static void renderMenu(OledDriver& d, int sel) {
  drawTitle(d, "-- MENU --");
  for (int i = 0; i < 5; ++i) {
    int y = 12 + i * 10;
    if (i == sel) { d.fillRect(0, y, 128, 10, true); d.drawString(2, y+1, kMenuItems[i], false); }
    else            d.drawString(2, y+1, kMenuItems[i], true);
  }
}

static void renderJointState(OledDriver& d, const DataSnapshot& sd, int& scroll) {
  drawTitle(d, "Joint State");
  const auto& J = sd.joints;
  int content_h = ((int(J.size()) + 1) / 2) * 17;
  scroll = std::clamp(scroll, 0, std::max(0, content_h - 54));
  for (int i = 0; i < (int)J.size() && i < 12; ++i) {
    int x = (i % 2) * 64, y = 13 + (i / 2) * 17 - scroll;
    if (y + 16 < 10 || y > 63) continue;
    d.drawString(x,    y,   J[i].name, true);
    d.drawString(x,    y+8, std::format("{:.1f}", J[i].pos),   true);
    d.drawString(x+36, y+8, std::format("{:.0f}C", J[i].temp), true);
  }
  drawScrollbar(d, scroll, content_h);
}

static void renderIMU(OledDriver& d, const DataSnapshot& sd, int& scroll) {
  drawTitle(d, "IMU State");
  constexpr int CONTENT_H = 60;
  scroll = std::clamp(scroll, 0, std::max(0, CONTENT_H - 54));
  const std::pair<int, std::string> rows[] = {
    {12, std::format("Gx{:+6.1f}", sd.imu_gx)},
    {22, std::format("Gy{:+6.1f}", sd.imu_gy)},
    {32, std::format("Gz{:+6.1f}", sd.imu_gz)},
    {42, std::format("Ax{:+5.2f}", sd.imu_ax)},
    {52, std::format("Ay{:+5.2f}", sd.imu_ay)},
    {62, std::format("Az{:+5.2f}", sd.imu_az)},
  };
  for (auto& [y, text] : rows) {
    int sy = y - scroll;
    if (sy >= 10 && sy < 64) d.drawString(0, sy, text, true);
  }
  constexpr int CX=100, CY=38, R=22;
  d.drawCircle(CX, CY, R, true);
  d.drawHLine(CX-R, CY, 2*R+1, true);
  d.drawVLine(CX, CY-R, 2*R+1, true);
  float nx = std::clamp(sd.imu_gy / 90.f, -1.f, 1.f);
  float ny = std::clamp(sd.imu_gx / 90.f, -1.f, 1.f);
  d.fillCircle(CX + (int)(nx*(R-3)), CY + (int)(ny*(R-3)), 3, true);
  drawScrollbar(d, scroll, CONTENT_H);
}

static void renderJoystick(OledDriver& d, const DataSnapshot& sd) {
  drawTitle(d, "Joystick");
  auto btn = [&](int bit) { return (sd.joy_buttons >> bit) & 1u; };
  auto drawBtn = [&](int x, int y, const char* label, bool pressed) {
    int w = d.textWidth(label) + 2;
    if (pressed) { d.fillRect(x-1, y-1, w, 9, true); d.drawString(x, y, label, false); }
    else           d.drawString(x, y, label, true);
  };
  drawBtn(1,   12, "LB", btn(4)); drawBtn(43,  12, "Bk", btn(7));
  drawBtn(69,  12, "St", btn(6)); drawBtn(114, 12, "RB", btn(5));
  constexpr int LX=22, LY=32, LR=14;
  d.drawCircle(LX, LY, LR, true);
  d.drawHLine(LX-LR, LY, 2*LR+1, true); d.drawVLine(LX, LY-LR, 2*LR+1, true);
  d.fillCircle(LX+(int)(sd.joy_lx*(LR-3)), LY+(int)(sd.joy_ly*(LR-3)), 3, true);
  constexpr int RX=82, RY=22, RR=8;
  d.drawCircle(RX, RY, RR, true);
  d.drawHLine(RX-RR, RY, 2*RR+1, true); d.drawVLine(RX, RY-RR, 2*RR+1, true);
  d.fillCircle(RX+(int)(sd.joy_rx*(RR-2)), RY+(int)(sd.joy_ry*(RR-2)), 2, true);
  drawBtn(107, 28, "Y", btn(3)); drawBtn(96, 37, "X", btn(2));
  drawBtn(116, 37, "B", btn(1)); drawBtn(107, 46, "A", btn(0));
  d.drawString(0,  56, std::format("L{:+.1f} {:+.1f}", sd.joy_lx, sd.joy_ly), true);
  d.drawString(66, 56, std::format("R{:+.1f} {:+.1f}", sd.joy_rx, sd.joy_ry), true);
}

static void renderLog(OledDriver& d, const DataSnapshot& sd, int& scroll) {
  drawTitle(d, "AimRT Log");
  constexpr int kRowH=9, kVisRows=6, kChars=21;
  const auto& lines = sd.log_lines;
  int total = (int)lines.size();
  int max_scroll = std::max(0, total - kVisRows);
  scroll = std::clamp(scroll, 0, max_scroll);
  int start = std::max(0, total - kVisRows - scroll);
  for (int i = 0; i < kVisRows && (start+i) < total; ++i) {
    std::string row = lines[start+i];
    if ((int)row.size() > kChars) row.resize(kChars);
    d.drawString(0, 12 + i*kRowH, row, true);
  }
  if (total > kVisRows) {
    int bar_h = std::max(6, kVisRows * 54 / total);
    int bar_y = 10 + (max_scroll > 0 ? scroll * (54 - bar_h) / max_scroll : 0);
    d.fillRect(126, bar_y, 2, bar_h, true);
  }
}

static void renderSBC(OledDriver& d, const DataSnapshot& sd, int& scroll) {
  drawTitle(d, "SBC Status");
  constexpr int CONTENT_H = 58;
  scroll = std::clamp(scroll, 0, std::max(0, CONTENT_H - 54));
  int ncores = (int)sd.cpu_core_pct.size();
  for (int i = 0; i < ncores && i < 8; ++i) {
    int x = (i / 4) * 64, y = 13 + (i % 4) * 8 - scroll;
    if (y < 10 || y > 63) continue;
    char lbl[4]; std::snprintf(lbl, sizeof(lbl), "C%d", i);
    d.drawString(x, y, lbl, true);
    d.drawBar(x+12, y, 48, 5, (int)sd.cpu_core_pct[i], true);
  }
  int y_bot = 45 - scroll;
  if (y_bot >= 10 && y_bot < 64) {
    d.drawString(0, y_bot, std::format("T:{:.0f}C", sd.cpu_temp_c), true);
    uint64_t used_kb = sd.ram_total_kb > sd.ram_avail_kb ? sd.ram_total_kb - sd.ram_avail_kb : 0;
    d.drawString(48, y_bot, std::format("RAM{:.1f}/{:.0f}G",
      used_kb/(1024.f*1024.f), sd.ram_total_kb/(1024.f*1024.f)), true);
  }
  if (y_bot+9 >= 10 && y_bot+9 < 64) {
    int pct = sd.ram_total_kb
            ? (int)(100.f*(sd.ram_total_kb-sd.ram_avail_kb)/sd.ram_total_kb) : 0;
    d.drawBar(0, y_bot+9, 128, 5, pct, true);
  }
  drawScrollbar(d, scroll, CONTENT_H);
}

static void renderUI(OledDriver& d, UIContext& ctx, const DataSnapshot& sd) {
  d.clear();
  if      (ctx.state == UIState::SLEEPING) { /* blank */ }
  else if (ctx.state == UIState::MAIN)     renderMain(d, sd);
  else if (ctx.state == UIState::MENU)     renderMenu(d, ctx.menu_sel);
  else {
    switch (ctx.screen_idx) {
      case 0: renderJointState(d, sd, ctx.scroll_offset); break;
      case 1: renderIMU       (d, sd, ctx.scroll_offset); break;
      case 2: renderJoystick  (d, sd);                    break;
      case 3: renderLog       (d, sd, ctx.scroll_offset); break;
      case 4: renderSBC       (d, sd, ctx.scroll_offset); break;
    }
  }
  d.display();
}

static void handleEvent(Event ev, UIContext& ctx, OledDriver& oled) {
  ctx.touch();
  if (ctx.state == UIState::SLEEPING) {
    ctx.state = UIState::MAIN;
    oled.setSleep(false);
    return;
  }
  switch (ctx.state) {
    case UIState::MAIN:
      if (ev == Event::ENC_CW || ev == Event::ENC_CCW || ev == Event::ENC_PUSH)
        ctx.state = UIState::MENU;
      break;
    case UIState::MENU:
      if      (ev == Event::ENC_CW)    ctx.menu_sel = (ctx.menu_sel + 1) % 5;
      else if (ev == Event::ENC_CCW)   ctx.menu_sel = (ctx.menu_sel + 4) % 5;
      else if (ev == Event::ENC_PUSH)  {
        ctx.screen_idx = ctx.menu_sel;
        ctx.state      = UIState::SCREEN;
        ctx.scroll_offset = 0;
      }
      else if (ev == Event::BTN_BACK)  ctx.state = UIState::MAIN;
      break;
    case UIState::SCREEN:
      if      (ev == Event::ENC_CW)    ctx.scroll_offset += 8;
      else if (ev == Event::ENC_CCW)   ctx.scroll_offset = std::max(0, ctx.scroll_offset - 8);
      else if (ev == Event::BTN_BACK || ev == Event::ENC_PUSH) {
        ctx.state = UIState::MENU;
        ctx.scroll_offset = 0;
      }
      break;
  }
}

// ============================================================================
//  GPIO polling thread  (std::thread riêng – cần sleep_for cực ngắn 500µs)
// ============================================================================
static void gpioThread(std::atomic<bool>& running, EventQueue& eq) {
  std::printf("[GPIO] Thread started.\n");
  try {
    InputPin clk(kPinClk), dt(kPinDt), sw(kPinSw), back(kPinBack);
    int lastClk = clk.Read(), lastSw = sw.Read(), lastBack = back.Read();

    while (running) {
      // Encoder rotation
      int curClk = clk.Read();
      if (curClk != lastClk && curClk == 0) {
        eq.push(dt.Read() != curClk ? Event::ENC_CCW : Event::ENC_CW);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      lastClk = curClk;

      // Encoder push
      int swState = sw.Read();
      if (swState != lastSw) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        int confirmed = sw.Read();
        if (confirmed == swState) {
          if (confirmed == 0) eq.push(Event::ENC_PUSH);
          lastSw = confirmed;
        }
      }

      // Back button
      int backState = back.Read();
      if (backState != lastBack) {
        if (backState == 0) { eq.push(Event::BTN_BACK); std::this_thread::sleep_for(50ms); }
        lastBack = backState;
      }

      std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "[GPIO CRITICAL]: %s\n", e.what());
    running = false;
  }
  std::printf("[GPIO] Thread stopped.\n");
}

// ============================================================================
//  SBC stats thread  (std::thread riêng – cần sleep ~1s mỗi vòng lặp)
// ============================================================================
static void statsThread(std::atomic<bool>& running, SharedData& sd) {
  while (running.load(std::memory_order_relaxed)) {
    auto cores = readCpuUsage();   // block ~200ms bên trong
    float temp = readCpuTemp();
    uint64_t total_kb, avail_kb;
    readRamInfo(total_kb, avail_kb);
    {
      std::lock_guard lock(sd.mtx);
      sd.cpu_core_pct = std::move(cores);
      sd.cpu_temp_c   = temp;
      sd.ram_total_kb = total_kb;
      sd.ram_avail_kb = avail_kb;
    }
    std::this_thread::sleep_for(800ms);
  }
}

// ============================================================================
//  OledUiModule::Initialize
// ============================================================================
bool OledUiModule::Initialize(aimrt::CoreRef core) {
  core_ = core;
  SetLogger(core_.GetLogger());
  subs_.clear();

  auto file_path = core_.GetConfigurator().GetConfigFilePath();
  try {
    if (!file_path.empty()) {
      YAML::Node cfg = YAML::LoadFile(file_path.data());

      ui_freq_               = cfg["ui_frequency"].as<int32_t>(20);
      sub_imu_topic_         = cfg["sub_imu_data_name"].as<std::string>("/imu/data");
      sub_joint_state_topic_ = cfg["sub_joint_state_name"].as<std::string>("/joint_states");
      sub_joy_topic_         = cfg["sub_joy_name"].as<std::string>("/joy");
    }

    // ── Subscriber: IMU ─────────────────────────────────────────────────
    subs_.push_back(core_.GetChannelHandle().GetSubscriber(sub_imu_topic_));
    bool ret = aimrt::channel::Subscribe<sensor_msgs::msg::Imu>(
      subs_.back(),
      [this](const std::shared_ptr<const sensor_msgs::msg::Imu>& msg) {
        OnImuData(msg);
      });
    AIMRT_CHECK_ERROR_THROW(ret, "Subscribe IMU failed.");

    // ── Subscriber: Joint State ─────────────────────────────────────────
    subs_.push_back(core_.GetChannelHandle().GetSubscriber(sub_joint_state_topic_));
    ret = aimrt::channel::Subscribe<sensor_msgs::msg::JointState>(
      subs_.back(),
      [this](const std::shared_ptr<const sensor_msgs::msg::JointState>& msg) {
        OnJointState(msg);
      });
    AIMRT_CHECK_ERROR_THROW(ret, "Subscribe JointState failed.");

    // ── Subscriber: Joystick ────────────────────────────────────────────
    subs_.push_back(core_.GetChannelHandle().GetSubscriber(sub_joy_topic_));
    ret = aimrt::channel::Subscribe<sensor_msgs::msg::Joy>(
      subs_.back(),
      [this](const std::shared_ptr<const sensor_msgs::msg::Joy>& msg) {
        OnJoyState(msg);
      });
    AIMRT_CHECK_ERROR_THROW(ret, "Subscribe Joy failed.");

    // ── Executor dùng để chạy MainLoop ─────────────────────────────────
    executor_ = core_.GetExecutorManager().GetExecutor("oled_ui_thread");
    AIMRT_CHECK_ERROR_THROW(executor_, "Can not get executor 'oled_ui_thread'.");

  } catch (const std::exception& e) {
    AIMRT_ERROR("Init failed: {}", e.what());
    return false;
  }

  // Khởi tạo dữ liệu mặc định
  sd_.robot_ip   = getLocalIP();
  sd_.robot_name = "MyBipedal By LongVu";
  sd_.pushLog("[INFO] OledUiModule started");
  sd_.pushLog("[INFO] Robot: MyBipedal");
  sd_.pushLog("[INFO] Waiting for sensor data...");

  AIMRT_INFO("Init succeeded. IMU={}, Joints={}, Joy={}, freq={}Hz",
    sub_imu_topic_, sub_joint_state_topic_, sub_joy_topic_, ui_freq_);
  return true;
}

// ============================================================================
//  OledUiModule::Start  –  đưa MainLoop vào AimRT executor
// ============================================================================
bool OledUiModule::Start() {
  try {
    executor_.Execute([this]() { MainLoop(); });
    AIMRT_INFO("Start succeeded.");
  } catch (const std::exception& e) {
    AIMRT_ERROR("Start failed: {}", e.what());
    return false;
  }
  return true;
}

// ============================================================================
//  OledUiModule::Shutdown
// ============================================================================
void OledUiModule::Shutdown() {
  run_flag_.store(false);
  AIMRT_INFO("Shutdown requested.");
}

// ============================================================================
//  Callbacks subscriber
// ============================================================================
void OledUiModule::OnImuData(const std::shared_ptr<const sensor_msgs::msg::Imu>& msg) {
  std::lock_guard lock(sd_.mtx);
  // Angular velocity (gyroscope) – đơn vị rad/s → hiển thị trực tiếp
  sd_.imu_gx = static_cast<float>(msg->angular_velocity.x);
  sd_.imu_gy = static_cast<float>(msg->angular_velocity.y);
  sd_.imu_gz = static_cast<float>(msg->angular_velocity.z);
  // Linear acceleration – đơn vị m/s²
  sd_.imu_ax = static_cast<float>(msg->linear_acceleration.x);
  sd_.imu_ay = static_cast<float>(msg->linear_acceleration.y);
  sd_.imu_az = static_cast<float>(msg->linear_acceleration.z);
}

void OledUiModule::OnJointState(
    const std::shared_ptr<const sensor_msgs::msg::JointState>& msg) {

  // Khởi tạo index map lần đầu
  if (!joint_index_initialized_ && !msg->name.empty()) {
    for (size_t i = 0; i < msg->name.size(); ++i)
      joint_state_index_map_[msg->name[i]] = static_cast<int>(i);
    joint_index_initialized_ = true;
    AIMRT_INFO("Joint index map initialised with {} joints.", msg->name.size());
  }

  std::lock_guard lock(sd_.mtx);

  // Đảm bảo vector joints đủ kích thước
  if (sd_.joints.size() != msg->name.size()) {
    sd_.joints.resize(msg->name.size());
    for (size_t i = 0; i < msg->name.size(); ++i)
      sd_.joints[i].name = msg->name[i];
  }

  for (size_t i = 0; i < msg->name.size(); ++i) {
    sd_.joints[i].name = msg->name[i];
    if (i < msg->position.size())
      sd_.joints[i].pos  = static_cast<float>(msg->position[i]);
    // JointState không có trường temperature chuẩn.
    // Nếu hardware publish effort[] để truyền nhiệt độ, dùng dòng dưới:
    // if (i < msg->effort.size()) sd_.joints[i].temp = static_cast<float>(msg->effort[i]);
    // Mặc định giữ nhiệt độ = 0 (hoặc giá trị trước đó).
  }
}

void OledUiModule::OnJoyState(
    const std::shared_ptr<const sensor_msgs::msg::Joy>& msg) {
  std::lock_guard lock(sd_.mtx);

  // sensor_msgs/Joy: axes[0]=LX, [1]=LY, [2]=RX, [3]=RY (tuỳ hardware)
  auto axis = [&](int i) -> float {
    return (i < (int)msg->axes.size()) ? msg->axes[i] : 0.f;
  };
  sd_.joy_lx = axis(0);
  sd_.joy_ly = axis(1);
  sd_.joy_rx = axis(2);
  sd_.joy_ry = axis(3);

  // Buttons → bitmask uint16_t
  uint16_t mask = 0;
  for (int i = 0; i < (int)msg->buttons.size() && i < 16; ++i)
    if (msg->buttons[i]) mask |= (1u << i);
  sd_.joy_buttons = mask;
}

// ============================================================================
//  MainLoop  –  chạy trên AimRT executor "oled_ui_thread"
// ============================================================================
bool OledUiModule::MainLoop() {
  AIMRT_INFO("MainLoop started on executor.");

  // ── Khởi tạo OLED ────────────────────────────────────────────────────────
  auto i2c = std::make_unique<I2CDevice>(kI2CDev, kOledAddr);
  if (!i2c->open()) {
    AIMRT_ERROR("Cannot open I2C device {}", kI2CDev);
    return false;
  }
  OledDriver oled(std::move(i2c));
  if (!oled.init()) {
    AIMRT_ERROR("OLED init failed");
    return false;
  }
  oled.clear(); oled.display();

  // ── GPIO thread & SBC stats thread ──────────────────────────────────────
  EventQueue eq;
  std::thread gpio_thr(gpioThread, std::ref(run_flag_), std::ref(eq));
  std::thread stats_thr(statsThread, std::ref(run_flag_), std::ref(sd_));

  // ── UI state machine ─────────────────────────────────────────────────────
  UIContext ctx;
  ctx.state = UIState::SLEEPING;
  oled.setSleep(true);

  const auto period = std::chrono::nanoseconds(1'000'000'000 / ui_freq_);

  try {
    while (run_flag_) {
      auto frame_start = std::chrono::steady_clock::now();

      // Đợi event tối đa 50ms (không block quá lâu để giữ nhịp render)
      auto ev_opt = eq.pop(std::chrono::milliseconds(50));

      if (ev_opt) {
        bool was_sleeping = (ctx.state == UIState::SLEEPING);
        handleEvent(*ev_opt, ctx, oled);
        if (was_sleeping) eq.clear();  // xoá bounce sau khi thức
      }

      // Auto-sleep khi không có thao tác
      if (ctx.state != UIState::SLEEPING && ctx.isTimedOut()) {
        AIMRT_INFO("Inactivity timeout – going to sleep.");
        ctx.state = UIState::SLEEPING;
        oled.setSleep(true);
        oled.clear(); oled.display();
        continue;
      }

      // Render frame
      if (ctx.state != UIState::SLEEPING) {
        DataSnapshot snap = sd_.snapshot();
        renderUI(oled, ctx, snap);
      }

      // Giữ nhịp frame theo ui_freq_
      auto elapsed = std::chrono::steady_clock::now() - frame_start;
      if (elapsed < period)
        std::this_thread::sleep_for(period - elapsed);
    }
  } catch (const std::exception& e) {
    AIMRT_ERROR("MainLoop exception: {}", e.what());
  }

  // Dọn dẹp
  run_flag_.store(false);
  gpio_thr.join();
  stats_thr.join();

  oled.setSleep(true);
  oled.clear(); oled.display();

  AIMRT_INFO("MainLoop exited.");
  return true;
}

}  // namespace mybipedal_deploy::oled_ui_module