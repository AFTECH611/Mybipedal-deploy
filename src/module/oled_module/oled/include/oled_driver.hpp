#pragma once

// ─────────────────────────────────────────────────────────────────────────────
//  oled_driver.hpp — SH1106 1.3" OLED driver stack  (AimRT-friendly edition)
//
//  Thay đổi so với bản gốc:
//    • Bỏ OledManager (class orchestrator + SCHED_FIFO render thread)
//    • Bỏ apply_rt_settings() / render_loop() / internal std::thread
//    • Thêm OledDisplay  — thin wrapper: I2cBus + SH1106 + Canvas gộp lại,
//      expose init() / tick() / update_*() để OledModule gọi từ executor
//    • RotaryEncoder giữ nguyên poll_thread_ riêng (IRQ-driven, nhẹ, bắt buộc)
//    • Tất cả Screen class giữ nguyên không đổi
//
//  Build deps:   libi2c-dev   libgpiod-dev
// ─────────────────────────────────────────────────────────────────────────────

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// libgpiod v2 opaque types (forward declarations)
struct gpiod_chip;
struct gpiod_line_request;
struct gpiod_edge_event_buffer;

namespace oled {

// ═══════════════════════════════════════════════════════════════════════════
//  Font
// ═══════════════════════════════════════════════════════════════════════════

struct Font {
    uint8_t        char_w;
    uint8_t        char_h;
    uint8_t        first;
    uint8_t        last;
    const uint8_t* data;
};

extern const Font FONT_5X7;
extern const Font FONT_4X6;

// ═══════════════════════════════════════════════════════════════════════════
//  I2cBus
// ═══════════════════════════════════════════════════════════════════════════

class I2cBus {
public:
    I2cBus(const std::string& device, uint8_t addr);
    ~I2cBus();
    I2cBus(const I2cBus&)            = delete;
    I2cBus& operator=(const I2cBus&) = delete;

    void write_cmd (std::initializer_list<uint8_t> cmds);
    void write_data(const uint8_t* data, std::size_t len);

private:
    int     fd_;
    uint8_t addr_;
};

// ═══════════════════════════════════════════════════════════════════════════
//  SH1106
// ═══════════════════════════════════════════════════════════════════════════

class SH1106 {
public:
    static constexpr int COLS       = 128;
    static constexpr int ROWS       = 64;
    static constexpr int PAGES      = 8;
    static constexpr int COL_OFFSET = 2;
    static constexpr int BUF_SIZE   = PAGES * COLS;

    explicit SH1106(I2cBus& bus);

    void init();
    void set_contrast(uint8_t val);
    void set_invert(bool inv);
    void set_power(bool on);

    void clear();
    void flush();
    void invalidate();

    void    set_pixel(int x, int y, bool on = true);
    bool    get_pixel(int x, int y) const;

    uint8_t*       buf()       { return back_; }
    const uint8_t* buf() const { return back_; }
    void           mark_dirty(int page);

private:
    I2cBus& bus_;
    uint8_t back_[BUF_SIZE]{};
    bool    dirty_[PAGES]{};

    void flush_page(int page);
};

// ═══════════════════════════════════════════════════════════════════════════
//  Canvas
// ═══════════════════════════════════════════════════════════════════════════

class Canvas {
public:
    explicit Canvas(SH1106& display);

    void clear()      { display_.clear();     }
    void flush()      { display_.flush();     }
    void invalidate() { display_.invalidate();}

    void pixel      (int x, int y, bool on = true);
    void hline      (int x, int y, int w, bool on = true);
    void vline      (int x, int y, int h, bool on = true);
    void line       (int x0, int y0, int x1, int y1, bool on = true);
    void rect       (int x, int y, int w, int h, bool on = true);
    void fill_rect  (int x, int y, int w, int h, bool on = true);
    void circle     (int cx, int cy, int r, bool on = true);
    void fill_circle(int cx, int cy, int r, bool on = true);

    int  text      (int x, int y, const std::string& s, const Font& f = FONT_5X7);
    int  text_inv  (int x, int y, const std::string& s, const Font& f = FONT_5X7);
    int  text_width(const std::string& s, const Font& f = FONT_5X7) const;

    void header      (const std::string& left, const std::string& right = "",
                      const Font& f = FONT_5X7);
    void separator   (int y);
    void progress_bar(int x, int y, int w, int h, uint8_t pct);
    void bar_chart   (int x, int y, int bar_w, int bar_h, int gap,
                      const std::vector<uint8_t>& values);
    void crosshair_dot(int cx, int cy, int r, float dx, float dy);
    void joystick_pad (int x, int y, int size, float sx, float sy);

private:
    SH1106& display_;

    void draw_char   (int x, int y, char c, const Font& f, bool inv);
    void mark_range_y(int y0, int y1);
};

// ═══════════════════════════════════════════════════════════════════════════
//  RotaryEncoder  — giữ nguyên (poll_thread_ là bắt buộc vì IRQ blocking)
// ═══════════════════════════════════════════════════════════════════════════

class RotaryEncoder {
public:
    struct Config {
        std::string gpiochip    = "/dev/gpiochip0";
        unsigned    pin_a       = 17;
        unsigned    pin_b       = 18;
        unsigned    pin_btn     = 27;
        int         debounce_ms = 50;
        bool        invert_dir  = false;
    };

    explicit RotaryEncoder(const Config& cfg);
    ~RotaryEncoder();
    RotaryEncoder(const RotaryEncoder&)            = delete;
    RotaryEncoder& operator=(const RotaryEncoder&) = delete;

    int  pop_delta();
    bool pop_press();

private:
    Config cfg_;

    // libgpiod v2 objects
    gpiod_chip*              chip_       = nullptr;
    gpiod_line_request*      req_ab_     = nullptr;  // A + B lines (input, edge on A)
    gpiod_line_request*      req_btn_    = nullptr;  // BTN line    (input, both edges)
    gpiod_edge_event_buffer* ev_buf_ab_  = nullptr;
    gpiod_edge_event_buffer* ev_buf_btn_ = nullptr;

    // A-line fd and B-line offset inside req_ab_
    // B is requested as plain input (no edge), read via get_value
    unsigned off_b_ = 0;  // offset of B within req_ab_ (index 1)

    std::atomic<int>  delta_  {0};
    std::atomic<bool> pressed_{false};
    std::atomic<bool> running_{false};
    std::thread       poll_thread_;

    int last_a_{0};
    std::chrono::steady_clock::time_point last_btn_tp_{};

    void poll_loop();
};

// ═══════════════════════════════════════════════════════════════════════════
//  Robot data structures
// ═══════════════════════════════════════════════════════════════════════════

struct JointState {
    std::string name;
    float       pos    = 0.f;
    float       vel    = 0.f;
    float       torque = 0.f;
    float       temp   = 0.f;
};

struct ImuState {
    float gx = 0.f, gy = 0.f, gz = 0.f;
    float ax = 0.f, ay = 0.f, az = 0.f;
    float roll = 0.f, pitch = 0.f;
};

struct JoyState {
    float       lx = 0.f, ly = 0.f;
    float       rx = 0.f, ry = 0.f;
    uint16_t    buttons = 0;
    std::string mode;
};

struct SbcStatus {
    std::array<uint8_t, 8> cpu_pct  {};
    uint8_t                ram_pct  = 0;
    float                  cpu_temp = 0.f;
    float                  load_1m  = 0.f;
};

struct RobotData {
    std::string             ip;
    std::string             mode;
    uint8_t                 battery  = 0;
    std::vector<JointState> joints;
    ImuState                imu;
    JoyState                joy;
    SbcStatus               sbc;
    std::deque<std::string> logs;
    static constexpr int    MAX_LOGS = 64;

    void push_log(std::string msg);
};

// ═══════════════════════════════════════════════════════════════════════════
//  Screen — abstract interface (không đổi)
// ═══════════════════════════════════════════════════════════════════════════

class Screen {
public:
    virtual ~Screen() = default;

    virtual const char* id()   const = 0;
    virtual const char* name() const = 0;
    virtual void render  (Canvas& c, const RobotData& d) = 0;
    virtual void on_encoder(int delta) {}
    virtual bool on_button() { return false; }
    virtual void on_enter()  {}
};

// ── Concrete screens ──────────────────────────────────────────────────────

class IpScreen : public Screen {
public:
    const char* id()   const override { return "IP"; }
    const char* name() const override { return "IP Address"; }
    void render(Canvas& c, const RobotData& d) override;
};

class MenuScreen : public Screen {
public:
    const char* id()   const override { return "MENU"; }
    const char* name() const override { return "Main Menu"; }
    void render   (Canvas& c, const RobotData& d) override;
    void on_encoder(int delta) override;
    void on_enter() override { sel_ = 0; }

    int  selected()        const { return sel_; }
    void set_count(int n)        { count_ = n;  }

private:
    int sel_   = 0;
    int count_ = 1;
};

class JointScreen : public Screen {
public:
    const char* id()   const override { return "JNTS"; }
    const char* name() const override { return "Joint State"; }
    void render   (Canvas& c, const RobotData& d) override;
    void on_encoder(int delta) override;
    void on_enter() override { scroll_ = 0; }

private:
    int scroll_ = 0;
};

class ImuScreen : public Screen {
public:
    const char* id()   const override { return "IMU"; }
    const char* name() const override { return "IMU State"; }
    void render(Canvas& c, const RobotData& d) override;
};

class JoyScreen : public Screen {
public:
    const char* id()   const override { return "JOY"; }
    const char* name() const override { return "Joystick"; }
    void render(Canvas& c, const RobotData& d) override;
};

class LogScreen : public Screen {
public:
    const char* id()   const override { return "LOG"; }
    const char* name() const override { return "AimRT Log"; }
    void render   (Canvas& c, const RobotData& d) override;
    void on_encoder(int delta) override;
    void on_enter() override { scroll_ = 0; }

private:
    int scroll_ = 0;
};

class SbcScreen : public Screen {
public:
    const char* id()   const override { return "SBC"; }
    const char* name() const override { return "SBC Status"; }
    void render(Canvas& c, const RobotData& d) override;
};

// ═══════════════════════════════════════════════════════════════════════════
//  OledDisplay — thay thế OledManager
//
//  Không có thread nội bộ nào ngoài RotaryEncoder::poll_thread_.
//  Toàn bộ render do OledModule::RenderLoop() (AimRT executor) điều khiển.
//
//  Lifecycle:
//    1. Construct
//    2. init()              — gọi một lần từ executor thread trước loop
//    3. update_*()          — gọi từ subscriber callbacks (bất kỳ thread nào)
//    4. tick()              — gọi mỗi frame từ executor thread
//    5. set_power(false)    — gọi khi Shutdown
// ═══════════════════════════════════════════════════════════════════════════

class OledDisplay {
public:
    struct Config {
        std::string           i2c_device  = "/dev/i2c-7";
        uint8_t               i2c_addr    = 0x3C;
        int                   fps         = 20;
        int                   boot_frames = 60;   // frames giữ IpScreen
        RotaryEncoder::Config encoder;
    };

    explicit OledDisplay(const Config& cfg);
    ~OledDisplay() = default;

    OledDisplay(const OledDisplay&)            = delete;
    OledDisplay& operator=(const OledDisplay&) = delete;

    // ── Gọi một lần từ executor thread trước khi bắt đầu tick() ─────────
    void init();

    // ── Gọi mỗi frame (executor thread) — xử lý encoder + render 1 frame ─
    //    Trả về false nếu display lỗi (OledModule dừng loop)
    bool tick();

    // ── Tắt màn hình khi shutdown ─────────────────────────────────────────
    void power_off();

    // ── Thread-safe data update API (subscriber callbacks) ────────────────
    void update_joints (const std::vector<JointState>& joints);
    void update_imu    (const ImuState&  imu);
    void update_joy    (const JoyState&  joy);
    void update_battery(uint8_t pct);
    void update_mode   (const std::string& mode);
    void update_ip     (const std::string& ip);
    void update_sbc    (const SbcStatus&  sbc);
    void push_log      (const std::string& msg);

private:
    // ── Hardware ──────────────────────────────────────────────────────────
    Config        cfg_;
    I2cBus        bus_;
    SH1106        display_;
    Canvas        canvas_;
    RotaryEncoder encoder_;

    // ── Data (guarded by data_mtx_) ───────────────────────────────────────
    RobotData          data_;
    mutable std::mutex data_mtx_;

    // ── Screens ───────────────────────────────────────────────────────────
    IpScreen    scr_ip_;
    MenuScreen  scr_menu_;
    JointScreen scr_joint_;
    ImuScreen   scr_imu_;
    JoyScreen   scr_joy_;
    LogScreen   scr_log_;
    SbcScreen   scr_sbc_;

    std::vector<Screen*> sub_screens_;
    Screen*              current_  = &scr_ip_;
    bool                 in_menu_  = false;
    int                  boot_cnt_ = 0;

    // ── FSM helpers (chỉ gọi từ executor thread — không cần mutex) ────────
    void handle_input();
};

} // namespace oled