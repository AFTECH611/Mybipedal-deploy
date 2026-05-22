// =============================================================================
//  main.cpp — OledDisplay full-feature example  (simulated data)
//
//  Mô phỏng đầy đủ một robot 6-khớp với:
//    • IMU lắc / nghiêng theo sóng sin
//    • 6 joints chuyển động theo profile hình sin lệch pha
//    • Joystick quét tròn + nút bấm ngẫu nhiên
//    • Battery discharge từ từ (1% mỗi 10 s)
//    • SBC: CPU load ngẫu nhiên, RAM drift, nhiệt độ dao động
//    • Logs tuôn ra mỗi 2 s
//    • Encoder/button được giả lập qua stdin (w/s/enter)
//
//  Build:
//    g++ -std=c++17 -O2 main.cpp oled_driver.cpp \
//        -li2c -lgpiod -lpthread -lm -o oled_demo
//
//  Chạy thử KHÔNG có phần cứng (mock mode):
//    Định nghĩa MOCK_HW ở compile time:
//    g++ -std=c++17 -O2 -DMOCK_HW main.cpp oled_driver.cpp ... -o oled_demo
//    Khi MOCK_HW được định nghĩa, I2cBus và RotaryEncoder sẽ không mở device thật;
//    toàn bộ logic simulator và FSM vẫn chạy, chỉ không ghi ra I2C/GPIO.
//
//  Target: Raspberry Pi với SH1106 1.3" OLED trên I2C-1 (0x3C)
//          Rotary encoder trên GPIO 17/18/27
// =============================================================================

#include "oled_driver.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

// ─── π shorthand ─────────────────────────────────────────────────────────────
static constexpr float PI = 3.14159265f;

// =============================================================================
//  Graceful shutdown
// =============================================================================
static std::atomic<bool> g_running{true};

static void sig_handler(int) { g_running.store(false); }

// =============================================================================
//  SimulatedSensors — tạo ra dữ liệu giả lập thực tế
// =============================================================================
class SimulatedSensors {
public:
    // ── Gọi từ simulator thread mỗi ~50 ms ──────────────────────────────────
    void step(float dt_s) {
        t_ += dt_s;

        _update_imu();
        _update_joints();
        _update_joy();
        _update_battery(dt_s);
        _update_sbc();
    }

    // ── Snapshot thread-safe ─────────────────────────────────────────────────
    oled::ImuState   imu()      const { std::lock_guard<std::mutex> lg(mtx_); return imu_; }
    std::vector<oled::JointState> joints() const { std::lock_guard<std::mutex> lg(mtx_); return joints_; }
    oled::JoyState   joy()      const { std::lock_guard<std::mutex> lg(mtx_); return joy_; }
    uint8_t          battery()  const { std::lock_guard<std::mutex> lg(mtx_); return battery_; }
    std::string      mode()     const { std::lock_guard<std::mutex> lg(mtx_); return mode_; }
    std::string      ip()       const { return "192.168.1.42"; }
    oled::SbcStatus  sbc()      const { std::lock_guard<std::mutex> lg(mtx_); return sbc_; }

    // ── Lấy log mới nhất (drain) ─────────────────────────────────────────────
    bool pop_log(std::string& out) {
        std::lock_guard<std::mutex> lg(log_mtx_);
        if (pending_logs_.empty()) return false;
        out = std::move(pending_logs_.front());
        pending_logs_.erase(pending_logs_.begin());
        return true;
    }

    // ── Giả lập encoder (gọi từ stdin-reader thread) ─────────────────────────
    void push_encoder_delta(int d) { enc_delta_.fetch_add(d); }
    void push_encoder_press()      { enc_pressed_.store(true); }

    int  pop_enc_delta()  { return enc_delta_.exchange(0); }
    bool pop_enc_press()  { return enc_pressed_.exchange(false); }

private:
    // ── Thời gian ────────────────────────────────────────────────────────────
    float t_ = 0.f;

    mutable std::mutex mtx_;

    // ── IMU ──────────────────────────────────────────────────────────────────
    oled::ImuState imu_;

    void _update_imu() {
        std::lock_guard<std::mutex> lg(mtx_);
        // Gyro: dao động nhỏ quanh 0, thêm drift chậm
        imu_.gx = 0.05f * std::sin(t_ * 1.3f) + 0.01f * std::sin(t_ * 7.1f);
        imu_.gy = 0.08f * std::sin(t_ * 0.9f + 1.f) + 0.02f * std::cos(t_ * 5.3f);
        imu_.gz = 0.03f * std::cos(t_ * 2.1f);
        // Accel: trọng lực (az ~ 9.81), thêm rung nhẹ
        imu_.ax = 0.3f * std::sin(t_ * 0.5f);
        imu_.ay = 0.2f * std::sin(t_ * 0.7f + 0.5f);
        imu_.az = 9.81f + 0.1f * std::sin(t_ * 3.f);
        // Roll/pitch từ nghiêng giả lập: robot đang bước chân
        imu_.roll  = 0.15f * std::sin(t_ * 2.f);
        imu_.pitch = 0.10f * std::sin(t_ * 2.f + PI / 4.f);
    }

    // ── Joints ───────────────────────────────────────────────────────────────
    std::vector<oled::JointState> joints_;

    void _update_joints() {
        static const std::string NAMES[] = {
            "LHP", "LKP", "LAP", "RHP", "RKP", "RAP"
        };
        // Biên độ tối đa mỗi khớp (rad)
        static const float AMP[] = {0.4f, 0.6f, 0.3f, 0.4f, 0.6f, 0.3f};
        // Phase lệch để tạo dáng bước
        static const float PHS[] = {0.f, PI/3.f, 2*PI/3.f, PI, 4*PI/3.f, 5*PI/3.f};
        // Tần số bước (0.5 Hz → chu kỳ 2 s)
        static const float FREQ = 0.5f;

        std::lock_guard<std::mutex> lg(mtx_);
        joints_.resize(6);
        for (int i = 0; i < 6; ++i) {
            auto& j   = joints_[i];
            j.name    = NAMES[i];
            j.pos     = AMP[i] * std::sin(2*PI * FREQ * t_ + PHS[i]);
            j.vel     = AMP[i] * 2*PI * FREQ * std::cos(2*PI * FREQ * t_ + PHS[i]);
            j.torque  = 5.f + 3.f * std::abs(j.vel);    // tỷ lệ với vận tốc
            j.temp    = 38.f + 12.f * (float)i / 5.f    // gradient nhiệt
                        + 2.f * std::sin(t_ * 0.1f + i);
        }
    }

    // ── Joystick ──────────────────────────────────────────────────────────────
    oled::JoyState joy_;

    void _update_joy() {
        std::lock_guard<std::mutex> lg(mtx_);
        // Left stick: quét hình elip chậm
        joy_.lx =  0.8f * std::cos(t_ * 0.6f);
        joy_.ly =  0.6f * std::sin(t_ * 0.6f);
        // Right stick: dao động nhỏ (camera giữ ổn định)
        joy_.rx =  0.1f * std::sin(t_ * 1.2f);
        joy_.ry = -0.1f * std::cos(t_ * 1.4f);
        // Nút: A nhấn khi t là số nguyên lẻ, B khi chẵn, v.v.
        int sec = static_cast<int>(t_);
        uint16_t btns = 0;
        if ((sec % 7) == 0) btns |= (1 << 0); // A
        if ((sec % 5) == 0) btns |= (1 << 1); // B
        if ((sec % 9) == 0) btns |= (1 << 2); // X
        if ((sec % 11)== 0) btns |= (1 << 3); // Y
        joy_.buttons = btns;

        // Mode thay đổi theo chu kỳ 15 s
        static const char* MODES[] = {"WALK", "TROT", "RUN", "STOP"};
        joy_.mode = mode_ = MODES[sec / 15 % 4];
    }

    std::string mode_;

    // ── Battery ───────────────────────────────────────────────────────────────
    uint8_t battery_ = 87;
    float   bat_acc_ = 0.f;

    void _update_battery(float dt_s) {
        // Discharge 1% mỗi 10 giây (để dễ nhìn)
        bat_acc_ += dt_s;
        if (bat_acc_ >= 10.f && battery_ > 0) {
            battery_--;
            bat_acc_ = 0.f;
        }
    }

    // ── SBC ──────────────────────────────────────────────────────────────────
    oled::SbcStatus sbc_;
    std::mt19937    rng_{42};

    void _update_sbc() {
        std::uniform_int_distribution<int> noise(-5, 5);
        std::lock_guard<std::mutex> lg(mtx_);
        // 8 core: base load dao động theo sin + nhiễu nhỏ
        for (int i = 0; i < 8; ++i) {
            float base = 40.f + 30.f * std::sin(t_ * 0.3f + i * PI / 4.f);
            int val = static_cast<int>(base) + noise(rng_);
            sbc_.cpu_pct[i] = static_cast<uint8_t>(std::clamp(val, 0, 100));
        }
        // RAM: drift từ từ lên 60-80%
        float ram = 55.f + 15.f * std::sin(t_ * 0.05f);
        sbc_.ram_pct = static_cast<uint8_t>(std::clamp((int)ram, 0, 100));
        // CPU temp: tỷ lệ với load
        sbc_.cpu_temp = 45.f + 0.3f * sbc_.cpu_pct[0] + 2.f * std::sin(t_ * 0.2f);
        // Load average 1m: mịn hơn
        sbc_.load_1m = 2.5f + 1.5f * std::sin(t_ * 0.15f);
    }

    // ── Pending logs ──────────────────────────────────────────────────────────
    std::mutex               log_mtx_;
    std::vector<std::string> pending_logs_;

    // ── Encoder ───────────────────────────────────────────────────────────────
    std::atomic<int>  enc_delta_{0};
    std::atomic<bool> enc_pressed_{false};

public:
    void emit_log(const std::string& msg) {
        std::lock_guard<std::mutex> lg(log_mtx_);
        pending_logs_.push_back(msg);
    }
};

// =============================================================================
//  LogGenerator — tạo log giả theo thời gian thực
// =============================================================================
class LogGenerator {
public:
    LogGenerator(SimulatedSensors& sensors) : sensors_(sensors) {}

    void run() {
        static const char* LEVELS[]   = {"[INFO]", "[WARN]", "[ERR] "};
        static const char* MODULES[]  = {"JointCtrl", "NavStack", "PowerMgr",
                                          "IMUFilter", "GaitPlan", "SafetyMon"};
        static const char* MESSAGES[] = {
            "pos ctrl OK",  "joint limit!", "encoder fault",
            "localize OK",  "obstacle det", "cmd timeout",
            "bat < 20%!",   "temp warning", "estop trigger",
            "gait updated", "step planned", "contact lost",
        };

        std::mt19937 rng(std::random_device{}());
        int idx = 0;
        while (g_running.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            if (!g_running.load()) break;

            std::uniform_int_distribution<int> lv(0, 2);
            std::uniform_int_distribution<int> mod(0, 5);
            std::uniform_int_distribution<int> msg(0, 11);

            char buf[32];
            std::snprintf(buf, sizeof(buf), "%s %s: %s",
                LEVELS[lv(rng)], MODULES[mod(rng)], MESSAGES[msg(rng)]);

            sensors_.emit_log(std::string(buf));
            idx++;
        }
    }

private:
    SimulatedSensors& sensors_;
};

// =============================================================================
//  StdinInput — đọc phím từ terminal để điều khiển encoder ảo
//              w = xoay lên, s = xoay xuống, enter/space = nhấn
// =============================================================================
class StdinInput {
public:
    StdinInput(SimulatedSensors& sensors) : sensors_(sensors) {}

    void run() {
        // Chỉ chạy nếu stdin là terminal thực
        if (!isatty(fileno(stdin))) return;

        // Đặt terminal vào non-canonical mode
        struct termios oldt, newt;
        if (tcgetattr(STDIN_FILENO, &oldt) != 0) return;
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        newt.c_cc[VMIN]  = 0;
        newt.c_cc[VTIME] = 1;  // 0.1 s timeout
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        while (g_running.load()) {
            char c = 0;
            if (read(STDIN_FILENO, &c, 1) == 1) {
                switch (c) {
                    case 'w': case 'W': sensors_.push_encoder_delta(-1); break;
                    case 's': case 'S': sensors_.push_encoder_delta(+1); break;
                    case '\n': case ' ': sensors_.push_encoder_press(); break;
                    case 'q': case 'Q': g_running.store(false); break;
                }
            }
        }

        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }

private:
    SimulatedSensors& sensors_;
};

// =============================================================================
//  main
// =============================================================================
int main(int argc, char* argv[]) {
    // ── Cài signal handler ────────────────────────────────────────────────────
    std::signal(SIGINT,  sig_handler);
    std::signal(SIGTERM, sig_handler);

    std::cout << "=================================================\n"
              << "  OLED Demo — SH1106 1.3\" / SH1106 on I2C-1\n"
              << "  Press Ctrl+C hoặc 'q' để thoát\n"
              << "  w/s = encoder xoay   Enter/Space = nhấn\n"
              << "=================================================\n"
              << std::flush;

    // ── Cấu hình OledDisplay ─────────────────────────────────────────────────
    oled::OledDisplay::Config cfg;
    cfg.i2c_device  = "/dev/i2c-1";
    cfg.i2c_addr    = 0x3C;
    cfg.fps         = 20;
    cfg.boot_frames = 60;   // 3 giây ở IpScreen (60 frames @ 20fps)

    cfg.encoder.gpiochip    = "/dev/gpiochip0";
    cfg.encoder.pin_a       = 17;
    cfg.encoder.pin_b       = 18;
    cfg.encoder.pin_btn     = 27;
    cfg.encoder.debounce_ms = 50;
    cfg.encoder.invert_dir  = false;

    // ── Khởi tạo OledDisplay ─────────────────────────────────────────────────
    std::unique_ptr<oled::OledDisplay> display;
    try {
        display = std::make_unique<oled::OledDisplay>(cfg);
        display->init();
        std::cout << "[main] OledDisplay init OK\n";
    } catch (const std::exception& e) {
        std::cerr << "[main] FATAL: OledDisplay init failed: " << e.what() << "\n"
                  << "       Kiểm tra:\n"
                  << "         - I2C enabled: sudo raspi-config → Interface → I2C\n"
                  << "         - OLED kết nối đúng SDA/SCL\n"
                  << "         - i2cdetect -y 1 hiển thị 0x3C\n"
                  << "         - gpiod cài đặt: sudo apt install libgpiod-dev\n";
        return 1;
    }

    // ── Sensor simulator ─────────────────────────────────────────────────────
    SimulatedSensors sensors;

    // Seeding IP và mode ban đầu ngay lập tức
    display->update_ip("192.168.1.42");
    display->update_mode("WALK");
    display->update_battery(87);

    // ── Thread: simulator vật lý (50 ms / bước) ──────────────────────────────
    std::thread sim_thread([&] {
        using clock = std::chrono::steady_clock;
        constexpr auto DT = std::chrono::milliseconds(50);
        constexpr float dt_s = 0.05f;

        auto next = clock::now() + DT;
        while (g_running.load()) {
            sensors.step(dt_s);

            // Đẩy data vào OledDisplay (thread-safe)
            display->update_imu(sensors.imu());
            display->update_joints(sensors.joints());
            display->update_joy(sensors.joy());
            display->update_battery(sensors.battery());
            display->update_mode(sensors.mode());
            display->update_sbc(sensors.sbc());

            // Drain log queue
            std::string log_msg;
            while (sensors.pop_log(log_msg))
                display->push_log(log_msg);

            std::this_thread::sleep_until(next);
            next += DT;
        }
    });

    // ── Thread: log generator ─────────────────────────────────────────────────
    LogGenerator log_gen(sensors);
    std::thread log_thread([&] { log_gen.run(); });

    // Emit vài log khởi động ngay lập tức
    sensors.emit_log("[INFO] NavStack: system init OK");
    sensors.emit_log("[INFO] JointCtrl: 6 joints online");
    sensors.emit_log("[INFO] GaitPlan: WALK mode loaded");
    sensors.emit_log("[INFO] IMUFilter: calibration done");

    // ── Thread: stdin keyboard ────────────────────────────────────────────────
    StdinInput kbd(sensors);
    std::thread kbd_thread([&] { kbd.run(); });

    // ── Render loop (main thread) ─────────────────────────────────────────────
    using clock = std::chrono::steady_clock;
    const auto frame_dur = std::chrono::microseconds(1'000'000 / cfg.fps);

    uint64_t frame_count  = 0;
    uint64_t error_count  = 0;
    auto     t_start      = clock::now();
    auto     t_next_frame = clock::now() + frame_dur;

    std::cout << "[main] Render loop bắt đầu @ " << cfg.fps << " fps\n";

    while (g_running.load()) {
        bool ok = display->tick();

        if (!ok) {
            ++error_count;
            std::cerr << "[main] tick() error #" << error_count
                      << " (I2C write fail?)\n";
            if (error_count > 10) {
                std::cerr << "[main] Quá nhiều lỗi, dừng.\n";
                break;
            }
        } else {
            error_count = 0;  // reset nếu frame OK
        }

        ++frame_count;

        // In stats mỗi 5 giây
        if (frame_count % (cfg.fps * 5) == 0) {
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                               clock::now() - t_start).count();
            float actual_fps = (elapsed > 0) ? (float)frame_count / elapsed : 0.f;

            // Lấy snapshot nhanh
            auto jnts = sensors.joints();
            float avg_temp = 0.f;
            for (auto& j : jnts) avg_temp += j.temp;
            if (!jnts.empty()) avg_temp /= jnts.size();

            auto imu = sensors.imu();

            std::cout << std::fixed << std::setprecision(1)
                      << "[stats] t=" << elapsed << "s"
                      << "  frames=" << frame_count
                      << "  fps=" << actual_fps
                      << "  bat=" << (int)sensors.battery() << "%"
                      << "  avg_joint_temp=" << avg_temp << "C"
                      << "  roll=" << (imu.roll * 180.f / PI) << "deg"
                      << "  pitch=" << (imu.pitch * 180.f / PI) << "deg"
                      << "\n" << std::flush;
        }

        // Ngủ đến frame kế
        std::this_thread::sleep_until(t_next_frame);
        t_next_frame += frame_dur;
    }

    // ── Cleanup ────────────────────────────────────────────────────────────────
    std::cout << "\n[main] Shutting down...\n";
    g_running.store(false);

    display->power_off();
    std::cout << "[main] OLED tắt.\n";

    if (sim_thread.joinable())  sim_thread.join();
    if (log_thread.joinable())  log_thread.join();
    if (kbd_thread.joinable())  kbd_thread.join();

    auto total_s = std::chrono::duration_cast<std::chrono::seconds>(
                       clock::now() - t_start).count();
    std::cout << "[main] Đã render " << frame_count << " frames trong "
              << total_s << " giây. Goodbye!\n";

    return 0;
}