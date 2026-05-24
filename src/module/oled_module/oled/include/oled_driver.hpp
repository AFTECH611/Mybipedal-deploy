#pragma once

#include <array>
#include <cstdint>
#include <initializer_list>
#include <memory>
#include <span>
#include <string_view>

#include "i2c.h"

// ---------------------------------------------------------------------------
//  OledDriver  –  SSD1306 128×64 framebuffer driver
//
//  All drawing calls operate on an in-memory framebuffer.
//  Call display() to flush to the physical display.
//
//  Coordinate system: x ∈ [0, 127], y ∈ [0, 63], (0,0) = top-left.
// ---------------------------------------------------------------------------
class OledDriver {
 public:
  static constexpr int WIDTH  = 128;
  static constexpr int HEIGHT = 64;
  static constexpr int FONT_W = 6;   // glyph stride including 1-px gap
  static constexpr int FONT_H = 8;

  explicit OledDriver(std::unique_ptr<I2CDevice> i2c);
  ~OledDriver() = default;

  // Non-copyable
  OledDriver(const OledDriver&)            = delete;
  OledDriver& operator=(const OledDriver&) = delete;

  // ── Lifecycle ─────────────────────────────────────────────────────────────
  bool init();

  // ── Display control ───────────────────────────────────────────────────────
  void setSleep(bool sleep);
  void setInvert(bool invert);
  void setContrast(uint8_t contrast);

  // ── Framebuffer ──────────────────────────────────────────────────────────
  void clear(bool on = false);   ///< fill all pixels on or off
  void display();                ///< flush framebuffer to OLED

  // ── Pixel ─────────────────────────────────────────────────────────────────
  void setPixel(int x, int y, bool on);
  [[nodiscard]] bool getPixel(int x, int y) const;

  // ── Primitives ────────────────────────────────────────────────────────────
  void drawHLine(int x, int y, int w, bool on = true);
  void drawVLine(int x, int y, int h, bool on = true);
  void drawLine (int x0, int y0, int x1, int y1, bool on = true);
  void drawRect (int x, int y, int w, int h, bool on = true);
  void fillRect (int x, int y, int w, int h, bool on = true);
  void drawCircle(int cx, int cy, int r, bool on = true);
  void fillCircle(int cx, int cy, int r, bool on = true);

  // ── Text ─────────────────────────────────────────────────────────────────
  void drawChar  (int x, int y, char c, bool on = true, uint8_t scale = 1);
  void drawString(int x, int y, std::string_view str, bool on = true, uint8_t scale = 1);
  [[nodiscard]] int textWidth(std::string_view str, uint8_t scale = 1) const;

  // ── Widgets ──────────────────────────────────────────────────────────────
  /// Horizontal progress bar [0–100 %] inside a border.
  void drawBar(int x, int y, int w, int h, int pct, bool on = true);

 private:
  bool sendCmd (uint8_t cmd);
  bool sendCmds(std::initializer_list<uint8_t> cmds);
  bool sendData(std::span<const uint8_t> data);

  std::unique_ptr<I2CDevice> i2c_;
  std::array<uint8_t, WIDTH * HEIGHT / 8> buf_{};
};
