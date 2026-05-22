// =============================================================================
//  main.cpp — OledDisplay standalone test  (tín hiệu tĩnh, không giả lập)
//
//  Mục đích: kiểm tra toàn bộ pipeline I2C + render + encoder FSM mà không
//            cần ROS2 hay AimRT.  Dữ liệu cố định, không có thread phụ nào
//            ngoài RotaryEncoder::poll_thread_ (bắt buộc vì libgpiod blocking).
//
//  Build:
//    g++ -std=c++17 -O2 main.cpp oled_driver.cpp \
//        -li2c -lgpiod -lpthread -lm -o oled_demo
//
//  Chạy:
//    sudo ./oled_demo            # cần quyền truy cập I2C + gpiochip
//    w/s  = encoder xoay   Enter/Space = nhấn   q = thoát
// =============================================================================

#include "oled_driver.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <termios.h>
#include <thread>
#include <unistd.h>

using namespace std::chrono;

// ─── Graceful shutdown ────────────────────────────────────────────────────────
static std::atomic<bool> g_running{true};
static void sig_handler(int) { g_running.store(false); }

// =============================================================================
//  StdinInput — đọc phím để giả lập encoder (không ảnh hưởng tín hiệu)
// =============================================================================
class StdinInput {
public:
    explicit StdinInput(oled::OledDisplay& display) : display_(display) {}

    void run() {
        if (!isatty(fileno(stdin))) return;

        struct termios oldt{}, newt{};
        if (tcgetattr(STDIN_FILENO, &oldt) != 0) return;
        newt          = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        newt.c_cc[VMIN]  = 0;
        newt.c_cc[VTIME] = 1;
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        while (g_running.load()) {
            char c = 0;
            if (read(STDIN_FILENO, &c, 1) == 1) {
                // Không có encoder thật → inject thẳng vào OledDisplay
                // thông qua update_joy để không phá vỡ tín hiệu tĩnh
                switch (c) {
                    // Encoder giả: OledDisplay::handle_input() đọc từ
                    // RotaryEncoder atomic — không thể inject trực tiếp từ
                    // bên ngoài.  Giải pháp: dùng RotaryEncoder mock hoặc
                    // dùng file descriptor giả.  Ở đây ta chỉ in hướng dẫn.
                    case 'w': case 'W':
                        std::cout << "[kbd] ↑ (xoay encoder vật lý)\n";
                        break;
                    case 's': case 'S':
                        std::cout << "[kbd] ↓ (xoay encoder vật lý)\n";
                        break;
                    case '\n': case ' ':
                        std::cout << "[kbd] OK (nhấn encoder vật lý)\n";
                        break;
                    case 'q': case 'Q':
                        g_running.store(false);
                        break;
                    default: break;
                }
            }
        }
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }

private:
    oled::OledDisplay& display_;
};

// =============================================================================
//  Tín hiệu tĩnh — tất cả dữ liệu được set một lần trước render loop
// =============================================================================
static void seed_static_data(oled::OledDisplay& display) {
    // ── IP / mode / battery ──────────────────────────────────────────────────
    display.update_ip("192.168.1.42");
    display.update_mode("WALK");
    display.update_battery(75);

    // ── IMU — robot đứng nghiêng nhẹ ────────────────────────────────────────
    oled::ImuState imu{};
    imu.gx    =  0.01f;   // rad/s — gần 0 khi đứng yên
    imu.gy    = -0.02f;
    imu.gz    =  0.00f;
    imu.ax    =  0.15f;   // m/s² — thành phần nghiêng nhỏ
    imu.ay    = -0.08f;
    imu.az    =  9.79f;   // gần g
    imu.roll  =  0.085f;  // rad ~ 4.9°
    imu.pitch = -0.050f;  // rad ~ 2.9°
    display.update_imu(imu);

    // ── Joints — 8 khớp biped ở tư thế đứng ─────────────────────────────────
    //   Đặt giá trị thực tế từ bài kiểm tra tư thế zero của robot
    std::vector<oled::JointState> joints = {
        //  name                    pos(rad)  vel(rad/s)  torque(Nm)  temp(°C)
        {"L_hip_roll",              0.00f,    0.00f,       1.2f,      42.3f},
        {"L_hip_pitch",            -0.12f,    0.00f,       8.5f,      44.1f},
        {"L_knee",                  0.28f,    0.00f,      12.3f,      47.8f},
        {"L_ankle",                -0.16f,    0.00f,       6.7f,      43.2f},
        {"R_hip_roll",              0.00f,    0.00f,       1.1f,      41.9f},
        {"R_hip_pitch",            -0.11f,    0.00f,       8.3f,      43.7f},
        {"R_knee",                  0.27f,    0.00f,      12.1f,      47.4f},
        {"R_ankle",                -0.15f,    0.00f,       6.5f,      42.8f},
    };
    display.update_joints(joints);

    // ── Joystick — lệnh đi thẳng chậm ───────────────────────────────────────
    oled::JoyState joy{};
    joy.ly      =  0.30f;   // forward 30%
    joy.lx      =  0.00f;   // không lateral
    joy.rx      =  0.00f;   // không yaw
    joy.ry      =  0.00f;
    joy.buttons =  0x00;    // không nút nào
    joy.mode    = "WALK";
    display.update_joy(joy);

    // ── SBC status ───────────────────────────────────────────────────────────
    oled::SbcStatus sbc{};
    //  cpu_pct[i] = load % từng core (đo thực tế hoặc ước tính)
    sbc.cpu_pct  = {38, 42, 35, 40, 37, 39, 33, 36};
    sbc.ram_pct  = 61;
    sbc.cpu_temp = 52.4f;   // °C
    sbc.load_1m  = 2.8f;
    display.update_sbc(sbc);

    // ── Log khởi động ────────────────────────────────────────────────────────
    display.push_log("[INFO] System: boot OK");
    display.push_log("[INFO] JointCtrl: 8 joints ready");
    display.push_log("[INFO] IMUFilter: calibrated");
    display.push_log("[INFO] GaitPlan: WALK loaded");
    display.push_log("[INFO] NavStack: standby");
}

// =============================================================================
//  main
// =============================================================================
int main() {
    std::signal(SIGINT,  sig_handler);
    std::signal(SIGTERM, sig_handler);

    std::cout << "=========================================\n"
              << "  OLED Static Test — SH1106 on I2C-7\n"
              << "  Radxa Rock 5B+  (libgpiod 1.6.x)\n"
              << "  q = thoát    Encoder vật lý để navigate\n"
              << "=========================================\n"
              << std::flush;

    // ── Cấu hình ─────────────────────────────────────────────────────────────
    oled::OledDisplay::Config cfg;
    cfg.i2c_device  = "/dev/i2c-7";   // Radxa 5B+: I2C7 = SDA pin35, SCL pin36
    cfg.i2c_addr    = 0x3C;
    cfg.fps         = 20;
    cfg.boot_frames = 40;              // 2s ở IpScreen (40 frames @ 20fps)

    // Encoder — Radxa 5B+ physical pin → gpiochip + line offset
    // Physical header pin 11/13/15 nằm trên GPIO bank nào cần xác nhận bằng:
    //   gpioinfo | grep -A5 "gpiochip"
    // Dưới đây dùng gpiochip3 và offset tương ứng (thay đổi nếu gpioinfo khác)
    cfg.encoder.gpiochip    = "/dev/gpiochip3";
    cfg.encoder.pin_a       = 13;  // physical pin 13 → offset trong gpiochip3
    cfg.encoder.pin_b       = 11;  // physical pin 11
    cfg.encoder.pin_btn     = 15;  // physical pin 15 (encoder push)
    cfg.encoder.debounce_ms = 50;
    cfg.encoder.invert_dir  = false;

    // ── Khởi tạo ─────────────────────────────────────────────────────────────
    std::unique_ptr<oled::OledDisplay> display;
    try {
        display = std::make_unique<oled::OledDisplay>(cfg);
        display->init();
        std::cout << "[main] OledDisplay init OK\n";
    } catch (const std::exception& e) {
        std::cerr << "[main] FATAL: " << e.what() << "\n"
                  << "       sudo i2cdetect -y 7  →  kiểm tra 0x3C\n"
                  << "       ls /dev/i2c-*  →  xác nhận i2c-7 tồn tại\n";
        return 1;
    }

    // ── Đổ dữ liệu tĩnh một lần duy nhất ────────────────────────────────────
    seed_static_data(*display);
    std::cout << "[main] Static data seeded.\n";

    // ── Stdin (không ảnh hưởng dữ liệu, chỉ để thoát) ───────────────────────
    StdinInput kbd(*display);
    std::thread kbd_thread([&] { kbd.run(); });

    // ── Render loop (main thread) ─────────────────────────────────────────────
    //    Chỉ gọi tick() đúng nhịp, không có bất kỳ update dữ liệu nào nữa.
    const auto frame_dur  = microseconds(1'000'000 / cfg.fps);
    auto       next_frame = steady_clock::now() + frame_dur;
    uint64_t   frames     = 0;
    auto       t_start    = steady_clock::now();

    std::cout << "[main] Render loop @ " << cfg.fps << " fps  (Ctrl+C / q để thoát)\n";

    while (g_running.load()) {
        if (!display->tick()) {
            std::cerr << "[main] tick() I2C error, retrying...\n";
            std::this_thread::sleep_for(milliseconds(100));
        } else {
            ++frames;
        }

        std::this_thread::sleep_until(next_frame);
        next_frame += frame_dur;
    }

    // ── Cleanup ───────────────────────────────────────────────────────────────
    display->power_off();

    if (kbd_thread.joinable()) kbd_thread.join();

    auto elapsed = duration_cast<seconds>(steady_clock::now() - t_start).count();
    std::cout << "[main] Done. " << frames << " frames in "
              << elapsed << "s  (avg "
              << (elapsed > 0 ? frames / elapsed : frames) << " fps).\n";

    return 0;
}