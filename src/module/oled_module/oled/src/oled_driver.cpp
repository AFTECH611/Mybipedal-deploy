// ─────────────────────────────────────────────────────────────────────────────
//  oled_driver.cpp — SH1106 OLED driver  (AimRT-friendly, no render thread)
//
//  Thay đổi so với bản gốc:
//    • Xóa OledManager hoàn toàn
//    • Thêm OledDisplay::init() / tick() / power_off() / update_sbc()
//    • Xóa apply_rt_settings(), render_loop(), start(), stop()
//    • Mọi thứ còn lại (Font, I2cBus, SH1106, Canvas, RotaryEncoder, Screens)
//      giữ nguyên 100% — không sửa một dòng logic nào
// ─────────────────────────────────────────────────────────────────────────────

#define _GNU_SOURCE
#include "oled_driver.hpp"

#include <algorithm>
#include <cassert>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <gpiod.h>

namespace oled {

// ═══════════════════════════════════════════════════════════════════════════
//  Font data  (không đổi)
// ═══════════════════════════════════════════════════════════════════════════

// clang-format off
static const uint8_t FONT5X7_DATA[] = {
    0x00,0x00,0x00,0x00,0x00, // 0x20  (space)
    0x00,0x00,0x5F,0x00,0x00, // 0x21  !
    0x00,0x07,0x00,0x07,0x00, // 0x22  "
    0x14,0x7F,0x14,0x7F,0x14, // 0x23  #
    0x24,0x2A,0x7F,0x2A,0x12, // 0x24  $
    0x23,0x13,0x08,0x64,0x62, // 0x25  %
    0x36,0x49,0x55,0x22,0x50, // 0x26  &
    0x00,0x05,0x03,0x00,0x00, // 0x27  '
    0x00,0x1C,0x22,0x41,0x00, // 0x28  (
    0x00,0x41,0x22,0x1C,0x00, // 0x29  )
    0x14,0x08,0x3E,0x08,0x14, // 0x2A  *
    0x08,0x08,0x3E,0x08,0x08, // 0x2B  +
    0x00,0x50,0x30,0x00,0x00, // 0x2C  ,
    0x08,0x08,0x08,0x08,0x08, // 0x2D  -
    0x00,0x60,0x60,0x00,0x00, // 0x2E  .
    0x20,0x10,0x08,0x04,0x02, // 0x2F  /
    0x3E,0x51,0x49,0x45,0x3E, // 0x30  0
    0x00,0x42,0x7F,0x40,0x00, // 0x31  1
    0x42,0x61,0x51,0x49,0x46, // 0x32  2
    0x21,0x41,0x45,0x4B,0x31, // 0x33  3
    0x18,0x14,0x12,0x7F,0x10, // 0x34  4
    0x27,0x45,0x45,0x45,0x39, // 0x35  5
    0x3C,0x4A,0x49,0x49,0x30, // 0x36  6
    0x01,0x71,0x09,0x05,0x03, // 0x37  7
    0x36,0x49,0x49,0x49,0x36, // 0x38  8
    0x06,0x49,0x49,0x29,0x1E, // 0x39  9
    0x00,0x36,0x36,0x00,0x00, // 0x3A  :
    0x00,0x56,0x36,0x00,0x00, // 0x3B  ;
    0x08,0x14,0x22,0x41,0x00, // 0x3C  <
    0x14,0x14,0x14,0x14,0x14, // 0x3D  =
    0x00,0x41,0x22,0x14,0x08, // 0x3E  >
    0x02,0x01,0x51,0x09,0x06, // 0x3F  ?
    0x32,0x49,0x79,0x41,0x3E, // 0x40  @
    0x7E,0x11,0x11,0x11,0x7E, // 0x41  A
    0x7F,0x49,0x49,0x49,0x36, // 0x42  B
    0x3E,0x41,0x41,0x41,0x22, // 0x43  C
    0x7F,0x41,0x41,0x22,0x1C, // 0x44  D
    0x7F,0x49,0x49,0x49,0x41, // 0x45  E
    0x7F,0x09,0x09,0x09,0x01, // 0x46  F
    0x3E,0x41,0x49,0x49,0x7A, // 0x47  G
    0x7F,0x08,0x08,0x08,0x7F, // 0x48  H
    0x00,0x41,0x7F,0x41,0x00, // 0x49  I
    0x20,0x40,0x41,0x3F,0x01, // 0x4A  J
    0x7F,0x08,0x14,0x22,0x41, // 0x4B  K
    0x7F,0x40,0x40,0x40,0x40, // 0x4C  L
    0x7F,0x02,0x0C,0x02,0x7F, // 0x4D  M
    0x7F,0x04,0x08,0x10,0x7F, // 0x4E  N
    0x3E,0x41,0x41,0x41,0x3E, // 0x4F  O
    0x7F,0x09,0x09,0x09,0x06, // 0x50  P
    0x3E,0x41,0x51,0x21,0x5E, // 0x51  Q
    0x7F,0x09,0x19,0x29,0x46, // 0x52  R
    0x46,0x49,0x49,0x49,0x31, // 0x53  S
    0x01,0x01,0x7F,0x01,0x01, // 0x54  T
    0x3F,0x40,0x40,0x40,0x3F, // 0x55  U
    0x1F,0x20,0x40,0x20,0x1F, // 0x56  V
    0x3F,0x40,0x38,0x40,0x3F, // 0x57  W
    0x63,0x14,0x08,0x14,0x63, // 0x58  X
    0x07,0x08,0x70,0x08,0x07, // 0x59  Y
    0x61,0x51,0x49,0x45,0x43, // 0x5A  Z
    0x00,0x7F,0x41,0x41,0x00, // 0x5B  [
    0x02,0x04,0x08,0x10,0x20, // 0x5C  back-slash
    0x00,0x41,0x41,0x7F,0x00, // 0x5D  ]
    0x04,0x02,0x01,0x02,0x04, // 0x5E  ^
    0x40,0x40,0x40,0x40,0x40, // 0x5F  _
    0x00,0x01,0x02,0x04,0x00, // 0x60  `
    0x20,0x54,0x54,0x54,0x78, // 0x61  a
    0x7F,0x48,0x44,0x44,0x38, // 0x62  b
    0x38,0x44,0x44,0x44,0x20, // 0x63  c
    0x38,0x44,0x44,0x48,0x7F, // 0x64  d
    0x38,0x54,0x54,0x54,0x18, // 0x65  e
    0x08,0x7E,0x09,0x01,0x02, // 0x66  f
    0x0C,0x52,0x52,0x52,0x3E, // 0x67  g
    0x7F,0x08,0x04,0x04,0x78, // 0x68  h
    0x00,0x44,0x7D,0x40,0x00, // 0x69  i
    0x20,0x40,0x44,0x3D,0x00, // 0x6A  j
    0x7F,0x10,0x28,0x44,0x00, // 0x6B  k
    0x00,0x41,0x7F,0x40,0x00, // 0x6C  l
    0x7C,0x04,0x18,0x04,0x78, // 0x6D  m
    0x7C,0x08,0x04,0x04,0x78, // 0x6E  n
    0x38,0x44,0x44,0x44,0x38, // 0x6F  o
    0x7C,0x14,0x14,0x14,0x08, // 0x70  p
    0x08,0x14,0x14,0x18,0x7C, // 0x71  q
    0x7C,0x08,0x04,0x04,0x08, // 0x72  r
    0x48,0x54,0x54,0x54,0x20, // 0x73  s
    0x04,0x3F,0x44,0x40,0x20, // 0x74  t
    0x3C,0x40,0x40,0x20,0x7C, // 0x75  u
    0x1C,0x20,0x40,0x20,0x1C, // 0x76  v
    0x3C,0x40,0x30,0x40,0x3C, // 0x77  w
    0x44,0x28,0x10,0x28,0x44, // 0x78  x
    0x0C,0x50,0x50,0x50,0x3C, // 0x79  y
    0x44,0x64,0x54,0x4C,0x44, // 0x7A  z
    0x00,0x08,0x36,0x41,0x00, // 0x7B  {
    0x00,0x00,0x7F,0x00,0x00, // 0x7C  |
    0x00,0x41,0x36,0x08,0x00, // 0x7D  }
    0x10,0x08,0x08,0x10,0x08, // 0x7E  ~
};

static const uint8_t FONT4X6_DATA[] = {
    0x00,0x00,0x00,0x00, // 0x20  (space)
    0x00,0x00,0x2F,0x00, // 0x21  !
    0x03,0x00,0x03,0x00, // 0x22  "
    0x0A,0x1F,0x0A,0x1F, // 0x23  #
    0x12,0x15,0x15,0x09, // 0x24  $
    0x11,0x08,0x04,0x11, // 0x25  %
    0x16,0x09,0x15,0x12, // 0x26  &
    0x00,0x03,0x00,0x00, // 0x27  '
    0x00,0x0E,0x11,0x00, // 0x28  (
    0x00,0x11,0x0E,0x00, // 0x29  )
    0x05,0x02,0x05,0x00, // 0x2A  *
    0x04,0x0E,0x04,0x00, // 0x2B  +
    0x00,0x18,0x08,0x00, // 0x2C  ,
    0x04,0x04,0x04,0x04, // 0x2D  -
    0x00,0x18,0x18,0x00, // 0x2E  .
    0x10,0x08,0x04,0x02, // 0x2F  /
    0x0E,0x11,0x11,0x0E, // 0x30  0
    0x00,0x12,0x1F,0x10, // 0x31  1
    0x12,0x19,0x15,0x12, // 0x32  2
    0x0A,0x15,0x15,0x0A, // 0x33  3
    0x0C,0x0A,0x1F,0x08, // 0x34  4
    0x0F,0x15,0x15,0x09, // 0x35  5
    0x0E,0x15,0x15,0x08, // 0x36  6
    0x01,0x19,0x05,0x03, // 0x37  7
    0x0A,0x15,0x15,0x0A, // 0x38  8
    0x02,0x15,0x15,0x0E, // 0x39  9
    0x00,0x0A,0x0A,0x00, // 0x3A  :
    0x00,0x1A,0x0A,0x00, // 0x3B  ;
    0x04,0x0A,0x11,0x00, // 0x3C  <
    0x0A,0x0A,0x0A,0x0A, // 0x3D  =
    0x00,0x11,0x0A,0x04, // 0x3E  >
    0x02,0x01,0x15,0x02, // 0x3F  ?
    0x0E,0x11,0x15,0x06, // 0x40  @
    0x1E,0x05,0x05,0x1E, // 0x41  A
    0x1F,0x15,0x15,0x0A, // 0x42  B
    0x0E,0x11,0x11,0x0A, // 0x43  C
    0x1F,0x11,0x11,0x0E, // 0x44  D
    0x1F,0x15,0x15,0x11, // 0x45  E
    0x1F,0x05,0x05,0x01, // 0x46  F
    0x0E,0x11,0x15,0x1C, // 0x47  G
    0x1F,0x04,0x04,0x1F, // 0x48  H
    0x00,0x11,0x1F,0x11, // 0x49  I
    0x08,0x10,0x11,0x0F, // 0x4A  J
    0x1F,0x04,0x0A,0x11, // 0x4B  K
    0x1F,0x10,0x10,0x10, // 0x4C  L
    0x1F,0x02,0x04,0x1F, // 0x4D  M
    0x1F,0x02,0x04,0x1F, // 0x4E  N
    0x0E,0x11,0x11,0x0E, // 0x4F  O
    0x1F,0x05,0x05,0x02, // 0x50  P
    0x0E,0x11,0x19,0x2E, // 0x51  Q
    0x1F,0x05,0x0D,0x12, // 0x52  R
    0x12,0x15,0x15,0x09, // 0x53  S
    0x01,0x1F,0x01,0x01, // 0x54  T
    0x0F,0x10,0x10,0x0F, // 0x55  U
    0x07,0x08,0x10,0x0F, // 0x56  V
    0x1F,0x10,0x0C,0x1F, // 0x57  W
    0x11,0x0A,0x04,0x11, // 0x58  X
    0x03,0x04,0x18,0x03, // 0x59  Y
    0x11,0x19,0x15,0x13, // 0x5A  Z
    0x00,0x1F,0x11,0x00, // 0x5B  [
    0x02,0x04,0x08,0x10, // 0x5C  back-slash
    0x00,0x11,0x1F,0x00, // 0x5D  ]
    0x02,0x01,0x02,0x00, // 0x5E  ^
    0x10,0x10,0x10,0x10, // 0x5F  _
    0x00,0x01,0x02,0x00, // 0x60  `
    0x08,0x15,0x15,0x1E, // 0x61  a
    0x1F,0x12,0x12,0x0C, // 0x62  b
    0x0C,0x12,0x12,0x00, // 0x63  c
    0x0C,0x12,0x12,0x1F, // 0x64  d
    0x0C,0x15,0x15,0x04, // 0x65  e
    0x04,0x1E,0x05,0x01, // 0x66  f
    0x04,0x15,0x15,0x0E, // 0x67  g
    0x1F,0x04,0x04,0x18, // 0x68  h
    0x00,0x10,0x1D,0x10, // 0x69  i
    0x10,0x10,0x1D,0x00, // 0x6A  j
    0x1F,0x04,0x0A,0x10, // 0x6B  k
    0x00,0x0F,0x10,0x00, // 0x6C  l
    0x1C,0x04,0x08,0x1C, // 0x6D  m
    0x1C,0x04,0x04,0x18, // 0x6E  n
    0x0C,0x12,0x12,0x0C, // 0x6F  o
    0x1C,0x0A,0x0A,0x04, // 0x70  p
    0x04,0x0A,0x0A,0x1C, // 0x71  q
    0x1C,0x04,0x02,0x02, // 0x72  r
    0x08,0x15,0x15,0x02, // 0x73  s
    0x04,0x0F,0x14,0x08, // 0x74  t
    0x0C,0x10,0x10,0x1C, // 0x75  u
    0x04,0x08,0x10,0x0C, // 0x76  v
    0x0C,0x10,0x08,0x1C, // 0x77  w
    0x12,0x0C,0x0C,0x12, // 0x78  x
    0x02,0x14,0x14,0x0E, // 0x79  y
    0x12,0x1A,0x16,0x12, // 0x7A  z
    0x00,0x04,0x1B,0x11, // 0x7B  {
    0x00,0x00,0x1F,0x00, // 0x7C  |
    0x11,0x1B,0x04,0x00, // 0x7D  }
    0x04,0x02,0x04,0x02, // 0x7E  ~
};
// clang-format on

const Font FONT_5X7 = {5, 7, 0x20, 0x7E, FONT5X7_DATA};
const Font FONT_4X6 = {4, 6, 0x20, 0x7E, FONT4X6_DATA};

// ═══════════════════════════════════════════════════════════════════════════
//  I2cBus  (không đổi)
// ═══════════════════════════════════════════════════════════════════════════

I2cBus::I2cBus(const std::string& device, uint8_t addr) : addr_(addr) {
    fd_ = ::open(device.c_str(), O_RDWR);
    if (fd_ < 0)
        throw std::runtime_error("I2cBus: cannot open " + device + ": " +
                                 std::strerror(errno));
    if (::ioctl(fd_, I2C_SLAVE, addr_) < 0)
        throw std::runtime_error("I2cBus: I2C_SLAVE ioctl failed");
}

I2cBus::~I2cBus() {
    if (fd_ >= 0) ::close(fd_);
}

void I2cBus::write_cmd(std::initializer_list<uint8_t> cmds) {
    std::vector<uint8_t> buf;
    buf.reserve(1 + cmds.size());
    buf.push_back(0x00);
    for (auto c : cmds) buf.push_back(c);
    if (::write(fd_, buf.data(), buf.size()) < 0)
        throw std::runtime_error("I2cBus: write_cmd failed");
}

void I2cBus::write_data(const uint8_t* data, std::size_t len) {
    std::vector<uint8_t> buf;
    buf.reserve(1 + len);
    buf.push_back(0x40);
    buf.insert(buf.end(), data, data + len);
    if (::write(fd_, buf.data(), buf.size()) < 0)
        throw std::runtime_error("I2cBus: write_data failed");
}

// ═══════════════════════════════════════════════════════════════════════════
//  SH1106  (không đổi)
// ═══════════════════════════════════════════════════════════════════════════

SH1106::SH1106(I2cBus& bus) : bus_(bus) {}

void SH1106::init() {
    bus_.write_cmd({
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
        0xAD, 0x8B, 0xA1, 0xC8, 0xDA, 0x12, 0x81, 0xCF,
        0xD9, 0x1F, 0xDB, 0x40, 0xA4, 0xA6, 0xAF,
    });
    invalidate();
    clear();
    flush();
}

void SH1106::set_contrast(uint8_t val) { bus_.write_cmd({0x81, val}); }
void SH1106::set_invert  (bool inv)    { bus_.write_cmd({static_cast<uint8_t>(inv ? 0xA7 : 0xA6)}); }
void SH1106::set_power   (bool on)     { bus_.write_cmd({static_cast<uint8_t>(on  ? 0xAF : 0xAE)}); }

void SH1106::clear() {
    std::memset(back_, 0, sizeof(back_));
    std::fill(std::begin(dirty_), std::end(dirty_), true);
}
void SH1106::flush() {
    for (int p = 0; p < PAGES; ++p)
        if (dirty_[p]) flush_page(p);
}
void SH1106::invalidate() {
    std::fill(std::begin(dirty_), std::end(dirty_), true);
}

void SH1106::set_pixel(int x, int y, bool on) {
    if (x < 0 || x >= COLS || y < 0 || y >= ROWS) return;
    int page = y / 8, bit = y % 8;
    if (on) back_[page * COLS + x] |=  (1u << bit);
    else    back_[page * COLS + x] &= ~(1u << bit);
    dirty_[page] = true;
}

bool SH1106::get_pixel(int x, int y) const {
    if (x < 0 || x >= COLS || y < 0 || y >= ROWS) return false;
    return (back_[(y/8) * COLS + x] >> (y%8)) & 1;
}

void SH1106::mark_dirty(int page) {
    if (page >= 0 && page < PAGES) dirty_[page] = true;
}

void SH1106::flush_page(int page) {
    const uint8_t col_lo = COL_OFFSET & 0x0F;
    const uint8_t col_hi = 0x10 | ((COL_OFFSET >> 4) & 0x0F);
    bus_.write_cmd({static_cast<uint8_t>(0xB0 | page), col_lo, col_hi});
    bus_.write_data(back_ + page * COLS, COLS);
    dirty_[page] = false;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Canvas  (không đổi)
// ═══════════════════════════════════════════════════════════════════════════

Canvas::Canvas(SH1106& display) : display_(display) {}

void Canvas::mark_range_y(int y0, int y1) {
    int p0 = std::max(0, y0 / 8);
    int p1 = std::min(SH1106::PAGES - 1, y1 / 8);
    for (int p = p0; p <= p1; ++p) display_.mark_dirty(p);
}

void Canvas::pixel (int x, int y, bool on) { display_.set_pixel(x, y, on); }

void Canvas::hline(int x, int y, int w, bool on) {
    for (int i = 0; i < w; ++i) display_.set_pixel(x + i, y, on);
}
void Canvas::vline(int x, int y, int h, bool on) {
    for (int i = 0; i < h; ++i) display_.set_pixel(x, y + i, on);
}

void Canvas::line(int x0, int y0, int x1, int y1, bool on) {
    int dx = std::abs(x1-x0), sx = x0 < x1 ? 1 : -1;
    int dy = std::abs(y1-y0), sy = y0 < y1 ? 1 : -1;
    int err = (dx > dy ? dx : -dy) / 2, e2;
    while (true) {
        display_.set_pixel(x0, y0, on);
        if (x0 == x1 && y0 == y1) break;
        e2 = err;
        if (e2 > -dx) { err -= dy; x0 += sx; }
        if (e2 <  dy) { err += dx; y0 += sy; }
    }
}

void Canvas::rect(int x, int y, int w, int h, bool on) {
    hline(x, y, w, on); hline(x, y+h-1, w, on);
    vline(x, y, h, on); vline(x+w-1, y, h, on);
}

void Canvas::fill_rect(int x, int y, int w, int h, bool on) {
    uint8_t* buf = display_.buf();
    int x1 = std::min(x+w, SH1106::COLS);
    int y1 = std::min(y+h, SH1106::ROWS);
    x = std::max(x, 0); y = std::max(y, 0);
    for (int row = y; row < y1; ++row) {
        int page = row/8, bit = row%8;
        for (int col = x; col < x1; ++col) {
            if (on) buf[page * SH1106::COLS + col] |=  (1u << bit);
            else    buf[page * SH1106::COLS + col] &= ~(1u << bit);
        }
    }
    mark_range_y(y, y1-1);
}

void Canvas::circle(int cx, int cy, int r, bool on) {
    int x = 0, y = r, d = 3 - 2*r;
    auto plot8 = [&](int px, int py) {
        display_.set_pixel(cx+px,cy+py,on); display_.set_pixel(cx-px,cy+py,on);
        display_.set_pixel(cx+px,cy-py,on); display_.set_pixel(cx-px,cy-py,on);
        display_.set_pixel(cx+py,cy+px,on); display_.set_pixel(cx-py,cy+px,on);
        display_.set_pixel(cx+py,cy-px,on); display_.set_pixel(cx-py,cy-px,on);
    };
    while (y >= x) {
        plot8(x, y);
        if (d < 0) d += 4*x+6; else { d += 4*(x-y)+10; --y; }
        ++x;
    }
}

void Canvas::fill_circle(int cx, int cy, int r, bool on) {
    for (int y = -r; y <= r; ++y)
        for (int x = -r; x <= r; ++x)
            if (x*x + y*y <= r*r)
                display_.set_pixel(cx+x, cy+y, on);
}

void Canvas::draw_char(int x, int y, char c, const Font& f, bool inv) {
    if (c < (char)f.first || c > (char)f.last) c = '?';
    const uint8_t* glyph = f.data + ((uint8_t)c - f.first) * f.char_w;
    uint8_t* buf = display_.buf();
    int page0 = y/8, shift = y%8;

    for (int col = 0; col < f.char_w; ++col) {
        int bx = x + col;
        if (bx < 0 || bx >= SH1106::COLS) continue;
        uint8_t g = glyph[col];
        if (page0 < SH1106::PAGES) {
            uint8_t lo = (g << shift);
            if (inv) buf[page0 * SH1106::COLS + bx] &= ~lo;
            else     buf[page0 * SH1106::COLS + bx] |=  lo;
            display_.mark_dirty(page0);
        }
        if (shift > 0 && page0+1 < SH1106::PAGES) {
            uint8_t hi = (g >> (8 - shift));
            if (inv) buf[(page0+1) * SH1106::COLS + bx] &= ~hi;
            else     buf[(page0+1) * SH1106::COLS + bx] |=  hi;
            display_.mark_dirty(page0+1);
        }
    }
}

int Canvas::text(int x, int y, const std::string& s, const Font& f) {
    for (char c : s) { draw_char(x, y, c, f, false); x += f.char_w + 1; }
    return x;
}

int Canvas::text_inv(int x, int y, const std::string& s, const Font& f) {
    fill_rect(x-1, y, text_width(s, f)+2, f.char_h+1, true);
    for (char c : s) { draw_char(x, y, c, f, true); x += f.char_w + 1; }
    return x;
}

int Canvas::text_width(const std::string& s, const Font& f) const {
    if (s.empty()) return 0;
    return static_cast<int>(s.size()) * (f.char_w + 1) - 1;
}

void Canvas::header(const std::string& left, const std::string& right, const Font& f) {
    fill_rect(0, 0, SH1106::COLS, 9, true);
    int x = 1;
    for (char c : left) { draw_char(x, 1, c, f, true); x += f.char_w + 1; }
    if (!right.empty()) {
        int rw = text_width(right, f);
        x = SH1106::COLS - rw - 2;
        for (char c : right) { draw_char(x, 1, c, f, true); x += f.char_w + 1; }
    }
}

void Canvas::separator(int y) { hline(0, y, SH1106::COLS, true); }

void Canvas::progress_bar(int x, int y, int w, int h, uint8_t pct) {
    rect(x, y, w, h, true);
    int fill = static_cast<int>((w-2) * std::min(pct,(uint8_t)100) / 100.f);
    if (fill > 0) fill_rect(x+1, y+1, fill, h-2, true);
}

void Canvas::bar_chart(int x, int y, int bar_w, int bar_h, int gap,
                       const std::vector<uint8_t>& values) {
    int bx = x;
    for (auto v : values) {
        int bh = static_cast<int>(bar_h * std::min(v,(uint8_t)100) / 100.f);
        rect(bx, y, bar_w, bar_h, true);
        if (bh > 0) fill_rect(bx+1, y+bar_h-bh, bar_w-2, bh, true);
        bx += bar_w + gap;
    }
}

void Canvas::crosshair_dot(int cx, int cy, int r, float dx, float dy) {
    circle(cx, cy, r, true);
    hline(cx-r+2, cy, 2*r-3, true);
    vline(cx, cy-r+2, 2*r-3, true);
    float mag = std::sqrt(dx*dx + dy*dy);
    if (mag > 1.f) { dx /= mag; dy /= mag; }
    fill_circle(cx + static_cast<int>(dx*(r-2)),
                cy + static_cast<int>(dy*(r-2)), 2, true);
}

void Canvas::joystick_pad(int x, int y, int size, float sx, float sy) {
    rect(x, y, size, size, true);
    int cx = x + size/2, cy = y + size/2;
    hline(cx-2, cy, 5, true);
    vline(cx, cy-2, 5, true);
    int ex = std::clamp(cx + static_cast<int>(sx*(size/2-2)), x+2, x+size-3);
    int ey = std::clamp(cy + static_cast<int>(sy*(size/2-2)), y+2, y+size-3);
    line(cx, cy, ex, ey, true);
    fill_circle(ex, ey, 2, true);
}

// ═══════════════════════════════════════════════════════════════════════════
//  RotaryEncoder  (không đổi — poll_thread_ là bắt buộc vì libgpiod blocking)
// ═══════════════════════════════════════════════════════════════════════════

RotaryEncoder::RotaryEncoder(const Config& cfg) : cfg_(cfg) {
    // 1. Mở gpiochip3 cho Encoder (A, B, BTN) và nút CONFIRM
    chip3_ = gpiod_chip_open(cfg_.gpiochip3.c_str());
    if (!chip3_) {
        throw std::runtime_error("RotaryEncoder: cannot open " + cfg_.gpiochip3);
    }

    line_a_   = gpiod_chip_get_line(chip3_, cfg_.pin_a);
    line_b_   = gpiod_chip_get_line(chip3_, cfg_.pin_b);
    line_btn_ = gpiod_chip_get_line(chip3_, cfg_.pin_btn);
    // Nếu bạn có thêm nút confirm trên chip3:
    // line_confirm_ = gpiod_chip_get_line(chip3_, cfg_.pin_confirm);

    // 2. Mở gpiochip4 cho nút BACK
    chip4_ = gpiod_chip_open(cfg_.gpiochip4.c_str());
    if (!chip4_) {
        // Nếu mở lỗi chip4, hãy giải phóng chip3 trước khi throw
        gpiod_chip_close(chip3_);
        throw std::runtime_error("RotaryEncoder: cannot open " + cfg_.gpiochip4);
    }

    line_back_ = gpiod_chip_get_line(chip4_, cfg_.pin_back);

    // 3. Cấu hình các chân Input / Interrupt Edge
    if (!line_a_ || !line_b_ || !line_btn_ || !line_back_) {
        if (chip3_) gpiod_chip_close(chip3_);
        if (chip4_) gpiod_chip_close(chip4_);
        throw std::runtime_error("RotaryEncoder: failed to get GPIO lines.");
    }

    // Đăng ký nhận sự kiện cạnh (Edge Events)
    if (gpiod_line_request_rising_edge_events(line_btn_, "oled_encoder_btn") < 0 ||
        gpiod_line_request_both_edges_events(line_a_, "oled_encoder_a") < 0 ||
        gpiod_line_request_input(line_b_, "oled_encoder_b") < 0 ||
        gpiod_line_request_rising_edge_events(line_back_, "oled_button_back") < 0) 
    {
        gpiod_chip_close(chip3_);
        gpiod_chip_close(chip4_);
        throw std::runtime_error("RotaryEncoder: failed to request line events/inputs.");
    }

    // Khởi chạy thread xử lý poll sự kiện
    running_ = true;
    poll_thread_ = std::thread(&RotaryEncoder::poll_loop, this);
}

RotaryEncoder::~RotaryEncoder() {
    running_.store(false);
    if (poll_thread_.joinable()) poll_thread_.join();
    if (line_btn_) gpiod_line_release(line_btn_);
    if (line_b_)   gpiod_line_release(line_b_);
    if (line_a_)   gpiod_line_release(line_a_);
    if (chip3_)     gpiod_chip_close(chip3_);
    if (chip4_) gpiod_chip_close(chip4_);
}
}

int  RotaryEncoder::pop_delta() { return delta_.exchange(0);   }
bool RotaryEncoder::pop_press() { return pressed_.exchange(false); }

void RotaryEncoder::poll_loop() {
    while (running_.load()) {
        struct timespec ts{0, 10'000'000L};  // 10 ms
        gpiod_line_bulk bulk, event_bulk;
        gpiod_line_bulk_init(&bulk);
        gpiod_line_bulk_add(&bulk, line_a_);
        gpiod_line_bulk_add(&bulk, line_btn_);

        int ret = gpiod_line_event_wait_bulk(&bulk, &ts, &event_bulk);
        if (ret <= 0) continue;

        int n = gpiod_line_bulk_num_lines(&event_bulk);
        for (int i = 0; i < n; ++i) {
            gpiod_line* ev_line = gpiod_line_bulk_get_line(&event_bulk, i);
            gpiod_line_event ev;
            if (gpiod_line_event_read(ev_line, &ev) < 0) continue;

            if (ev_line == line_a_) {
                int curr_a = (ev.event_type == GPIOD_LINE_EVENT_RISING_EDGE) ? 1 : 0;
                int curr_b = gpiod_line_get_value(line_b_);
                if (curr_a == 0) {
                    int d = (curr_b == 1) ? 1 : -1;
                    if (cfg_.invert_dir) d = -d;
                    delta_.fetch_add(d);
                }
                last_a_ = curr_a;
            } else if (ev_line == line_btn_) {
                if (ev.event_type == GPIOD_LINE_EVENT_FALLING_EDGE) {
                    auto now = std::chrono::steady_clock::now();
                    auto gap = std::chrono::duration_cast<std::chrono::milliseconds>(
                                   now - last_btn_tp_).count();
                    if (gap >= cfg_.debounce_ms) {
                        pressed_.store(true);
                        last_btn_tp_ = now;
                    }
                }
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  RobotData
// ═══════════════════════════════════════════════════════════════════════════

void RobotData::push_log(std::string msg) {
    logs.push_back(std::move(msg));
    while ((int)logs.size() > MAX_LOGS) logs.pop_front();
}

// ═══════════════════════════════════════════════════════════════════════════
//  Screen implementations  (không đổi)
// ═══════════════════════════════════════════════════════════════════════════

void IpScreen::render(Canvas& c, const RobotData& d) {
    c.clear();
    c.fill_rect(0, 0, 128, 9, true);
    const char* title = "ROBOT ONLINE";
    int tw = c.text_width(title, FONT_5X7);
    int px = (128 - tw) / 2;
    for (char ch : std::string(title))
        px = c.text_inv(px, 1, std::string(1, ch), FONT_5X7);
    c.separator(10);
    c.text(2, 13, "IP: " + (d.ip.empty() ? "resolving..." : d.ip),   FONT_5X7);
    c.text(2, 22, "Mode: " + (d.mode.empty() ? "-" : d.mode),         FONT_5X7);
    char bat[12]; std::snprintf(bat, sizeof(bat), "Bat: %d%%", d.battery);
    c.text(2, 31, bat, FONT_5X7);
    c.separator(41);
    c.text(2, 44, "Preempt-RT active",    FONT_4X6);
    c.text(2, 51, "Push encoder to enter", FONT_4X6);
    static int blink = 0;
    if ((blink++ / 15) % 2 == 0) c.text(57, 57, "v", FONT_4X6);
}

void MenuScreen::on_encoder(int delta) {
    sel_ = (sel_ + delta % count_ + count_) % count_;
}
void MenuScreen::render(Canvas& c, const RobotData& d) {
    c.clear();
    c.header("MENU", "OK");
    static const char* labels[] = {
        "1 Joint State", "2 IMU State", "3 Joystick", "4 Log", "5 SBC Status",
    };
    int visible = std::min(count_, 5);
    int start   = std::max(0, sel_ - 4);
    for (int i = 0; i < visible && start+i < count_; ++i) {
        int idx = start + i, y = 11 + i * 10;
        if (idx == sel_) {
            c.fill_rect(0, y-1, 128, 9, true);
            int x = 2;
            for (char ch : std::string(labels[idx]))
                x = c.text_inv(x, y, std::string(1, ch), FONT_5X7);
        } else {
            c.text(2, y, labels[idx], FONT_5X7);
        }
    }
    char bat[8]; std::snprintf(bat, sizeof(bat), "%3d%%", d.battery);
    c.progress_bar(2, 57, 80, 6, d.battery);
    c.text(86, 57, bat, FONT_4X6);
}

void JointScreen::on_encoder(int delta) { scroll_ = std::max(0, scroll_ + delta); }
void JointScreen::render(Canvas& c, const RobotData& d) {
    c.clear();
    c.header("JOINTS", "1");
    c.text(2, 11, "Name  Pos    Temp", FONT_4X6);
    c.separator(18);
    constexpr int ROW_H = 8, ROW_START = 20;
    constexpr int ROWS_VIS = (SH1106::ROWS - ROW_START) / ROW_H;
    int total = static_cast<int>(d.joints.size());
    scroll_   = std::clamp(scroll_, 0, std::max(0, total - ROWS_VIS));
    for (int i = 0; i < ROWS_VIS && scroll_+i < total; ++i) {
        const auto& j = d.joints[scroll_+i];
        char buf[22];
        std::snprintf(buf, sizeof(buf), "%-5s%+6.1f %4.1fC",
                      j.name.c_str(), j.pos * 180.f / 3.14159f, j.temp);
        c.text(2, ROW_START + i*ROW_H, buf, FONT_4X6);
    }
    if (total > ROWS_VIS) {
        if (scroll_ > 0)                c.text(122, 19, "^", FONT_4X6);
        if (scroll_+ROWS_VIS < total)   c.text(122, 57, "v", FONT_4X6);
    }
}

void ImuScreen::render(Canvas& c, const RobotData& d) {
    c.clear();
    c.header("IMU", "2");
    char buf[22];
    const auto& m = d.imu;
    std::snprintf(buf, sizeof(buf), "Gx%+6.2f", m.gx); c.text(2, 11, buf, FONT_4X6);
    std::snprintf(buf, sizeof(buf), "Gy%+6.2f", m.gy); c.text(2, 18, buf, FONT_4X6);
    std::snprintf(buf, sizeof(buf), "Gz%+6.2f", m.gz); c.text(2, 25, buf, FONT_4X6);
    c.separator(33);
    std::snprintf(buf, sizeof(buf), "Ro%+6.1fd", m.roll  * 180.f / 3.14159f); c.text(2, 35, buf, FONT_4X6);
    std::snprintf(buf, sizeof(buf), "Pi%+6.1fd", m.pitch * 180.f / 3.14159f); c.text(2, 42, buf, FONT_4X6);
    constexpr int CX = 100, CY = 38, R = 20;
    c.crosshair_dot(CX, CY, R,
        std::clamp(m.roll  / 0.52f, -1.f, 1.f),
        std::clamp(m.pitch / 0.52f, -1.f, 1.f));
    c.text(CX-4, CY-R-6, "Ri", FONT_4X6);
    c.text(CX-4, CY+R+1, "Le", FONT_4X6);
}

void JoyScreen::render(Canvas& c, const RobotData& d) {
    c.clear();
    c.header("JOYSTICK", "3");
    const auto& j = d.joy;
    constexpr int PAD_SZ = 30;
    c.joystick_pad(2, 11, PAD_SZ, j.lx, j.ly);
    c.text(4, 42, "LX", FONT_4X6); c.text(16, 42, "LY", FONT_4X6);
    char buf[22];
    std::snprintf(buf, sizeof(buf), "%+.2f %+.2f", j.lx, j.ly);
    c.text(2, 50, buf, FONT_4X6);
    c.vline(43, 11, 53, true);
    static const char* BTN_NAMES[] = {"A","B","X","Y","L","R"};
    for (int i = 0; i < 6; ++i) {
        int bx = 47 + (i%3)*27, by = 13 + (i/3)*12;
        bool pressed = (j.buttons >> i) & 1;
        if (pressed) {
            c.fill_rect(bx, by, 20, 9, true);
            int tx = bx + (20 - (int)strlen(BTN_NAMES[i])*5) / 2;
            for (const char* p = BTN_NAMES[i]; *p; ++p)
                c.text_inv(tx, by+1, std::string(1, *p), FONT_4X6);
        } else {
            c.rect(bx, by, 20, 9, true);
            c.text(bx + (20 - (int)strlen(BTN_NAMES[i])*5)/2, by+1, BTN_NAMES[i], FONT_4X6);
        }
    }
    c.separator(38);
    c.text(47, 40, "Mode:", FONT_4X6);
    c.text(76, 40, j.mode.empty() ? "-" : j.mode, FONT_4X6);
    c.joystick_pad(108, 45, 18, j.rx, j.ry);
}

void LogScreen::on_encoder(int delta) { scroll_ = std::max(0, scroll_ - delta); }
void LogScreen::render(Canvas& c, const RobotData& d) {
    c.clear();
    c.header("AIMRT LOG", "4");
    constexpr int ROW_H = 8, ROWS_VIS = (SH1106::ROWS - 10) / ROW_H;
    const auto& logs = d.logs;
    int total = static_cast<int>(logs.size());
    scroll_   = std::clamp(scroll_, 0, std::max(0, total - ROWS_VIS));
    int start = total - ROWS_VIS - scroll_;
    for (int i = 0; i < ROWS_VIS; ++i) {
        int idx = start + i;
        if (idx < 0 || idx >= total) continue;
        c.text(2, 10 + i*ROW_H, logs[idx].substr(0, 21), FONT_4X6);
    }
    if (scroll_ > 0)              c.text(122, 11, "^", FONT_4X6);
    if (scroll_ < total-ROWS_VIS) c.text(122, 56, "v", FONT_4X6);
}

void SbcScreen::render(Canvas& c, const RobotData& d) {
    c.clear();
    c.header("SBC STATUS", "5");
    const auto& s = d.sbc;
    int avg = 0;
    for (auto v : s.cpu_pct) avg += v;
    avg /= 8;
    char buf[24];
    std::snprintf(buf, sizeof(buf), "CPU %2d%%  T:%.1fC  L:%.1f",
                  avg, s.cpu_temp, s.load_1m);
    c.text(2, 11, buf, FONT_4X6);
    c.progress_bar(2, 18, 124, 4, static_cast<uint8_t>(avg));
    std::snprintf(buf, sizeof(buf), "RAM %2d%%", s.ram_pct);
    c.text(2, 25, buf, FONT_4X6);
    c.progress_bar(2, 32, 124, 4, s.ram_pct);
    c.separator(39);
    std::vector<uint8_t> cores(s.cpu_pct.begin(), s.cpu_pct.end());
    c.bar_chart(2, 41, 12, 18, 3, cores);
    for (int i = 0; i < 8; ++i) {
        char lbl[2] = {static_cast<char>('0'+i), 0};
        c.text(3 + i*15, 60, lbl, FONT_4X6);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  OledDisplay  (thay thế OledManager)
// ═══════════════════════════════════════════════════════════════════════════

OledDisplay::OledDisplay(const Config& cfg)
    : cfg_    (cfg),
      bus_    (cfg.i2c_device, cfg.i2c_addr),
      display_(bus_),
      canvas_ (display_),
      encoder_(cfg.encoder) {
    sub_screens_ = {&scr_joint_, &scr_imu_, &scr_joy_, &scr_log_, &scr_sbc_};
    scr_menu_.set_count(static_cast<int>(sub_screens_.size()));
}

// ── Gọi một lần từ executor thread, trước tick() ─────────────────────────
void OledDisplay::init() {
    display_.init();   // gửi lệnh khởi tạo SH1106 qua I2C
}

// ── Gọi mỗi frame từ executor thread ─────────────────────────────────────
bool OledDisplay::tick() {
    try {
        // 1. Xử lý encoder input (non-blocking, chỉ đọc atomic)
        handle_input();

        // 2. Snapshot dữ liệu được bảo vệ bởi mutex
        RobotData snap;
        {
            std::lock_guard<std::mutex> lk(data_mtx_);
            snap = data_;
        }

        // 3. Render frame hiện tại lên back-buffer
        canvas_.clear();
        current_->render(canvas_, snap);

        // 4. Flush các page bẩn ra I2C
        canvas_.flush();

        ++boot_cnt_;
        return true;
    } catch (const std::exception& e) {
        // I2C error — báo lên để module dừng loop
        (void)e;
        return false;
    }
}

// ── Tắt màn hình khi Shutdown ─────────────────────────────────────────────
void OledDisplay::power_off() {
    display_.set_power(false);
}

// ── FSM: xử lý encoder — chỉ gọi từ executor thread ─────────────────────
void OledDisplay::handle_input() {
    int  delta   = encoder_.pop_delta();   // atomic, non-blocking
    bool pressed = encoder_.pop_press();   // atomic, non-blocking

    if (current_ == &scr_ip_) {
        if (pressed || boot_cnt_ >= cfg_.boot_frames) {
            current_ = &scr_menu_;
            in_menu_ = true;
            scr_menu_.on_enter();
        }
    } else if (in_menu_) {
        if (delta)   scr_menu_.on_encoder(delta);
        if (pressed) {
            in_menu_ = false;
            current_ = sub_screens_[scr_menu_.selected()];
            current_->on_enter();
        }
    } else {
        if (delta)   current_->on_encoder(delta);
        if (pressed && !current_->on_button()) {
            current_ = &scr_menu_;
            in_menu_ = true;
        }
    }
}

// ── Thread-safe update API ────────────────────────────────────────────────

void OledDisplay::update_joints(const std::vector<JointState>& joints) {
    std::lock_guard<std::mutex> lk(data_mtx_);
    data_.joints = joints;
}
void OledDisplay::update_imu(const ImuState& imu) {
    std::lock_guard<std::mutex> lk(data_mtx_);
    data_.imu = imu;
}
void OledDisplay::update_joy(const JoyState& joy) {
    std::lock_guard<std::mutex> lk(data_mtx_);
    data_.joy = joy;
}
void OledDisplay::update_battery(uint8_t pct) {
    std::lock_guard<std::mutex> lk(data_mtx_);
    data_.battery = std::min(pct, (uint8_t)100);
}
void OledDisplay::update_mode(const std::string& mode) {
    std::lock_guard<std::mutex> lk(data_mtx_);
    data_.mode = mode;
    data_.joy.mode = mode;   // JoyScreen cũng hiển thị mode
}
void OledDisplay::update_ip(const std::string& ip) {
    std::lock_guard<std::mutex> lk(data_mtx_);
    data_.ip = ip;
}
void OledDisplay::update_sbc(const SbcStatus& sbc) {
    std::lock_guard<std::mutex> lk(data_mtx_);
    data_.sbc = sbc;
}
void OledDisplay::push_log(const std::string& msg) {
    std::lock_guard<std::mutex> lk(data_mtx_);
    data_.push_log(msg);
}

} // namespace oled