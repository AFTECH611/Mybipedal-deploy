// =============================================================================
//  example/main.cpp  –  Standalone OLED SSD1306 test (no AimRT)
//
//  Exercises the OledDriver and I2CDevice without any ROS2 / AimRT dependency.
//  Useful for hardware bringup and visual sanity-checks on Radxa 5B+.
//
//  Build:  (handled by oled/example/CMakeLists.txt)
//    cmake -B build -S . && cmake --build build
//    sudo ./build/oled_example
// =============================================================================

#include "oled_driver.hpp"
#include "i2c.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <format>
#include <thread>

using namespace std::chrono_literals;

static constexpr const char* kI2CDev   = "/dev/i2c-7";
static constexpr uint8_t     kOledAddr = 0x3C;

int main() {
  std::printf("=== OLED SSD1306 standalone example (%s @ 0x%02X) ===\n",
              kI2CDev, kOledAddr);

  // ── Init ──────────────────────────────────────────────────────────────────
  auto i2c = std::make_unique<I2CDevice>(kI2CDev, kOledAddr);
  if (!i2c->open()) {
    std::fprintf(stderr, "FATAL: cannot open %s\n", kI2CDev);
    return 1;
  }
  OledDriver oled(std::move(i2c));
  if (!oled.init()) {
    std::fprintf(stderr, "FATAL: SSD1306 init failed\n");
    return 1;
  }

  // ── Splash ────────────────────────────────────────────────────────────────
  oled.clear();
  oled.drawString(10, 0,  "MyBipedal", true, 2);
  oled.drawHLine(0, 18, 128, true);
  oled.drawString(0, 22, "OLED driver OK", true);
  oled.drawString(0, 32, std::format("I2C: {}", kI2CDev), true);
  oled.drawBar(0, 50, 128, 8, 75, true);
  oled.display();
  std::this_thread::sleep_for(2s);

  // ── Primitives demo ───────────────────────────────────────────────────────
  for (int frame = 0; frame < 60; ++frame) {
    oled.clear();
    int cx = 20 + frame % 88;
    oled.drawCircle(cx, 32, 12, true);
    oled.fillCircle(cx, 32, 4, true);
    oled.drawRect(0, 0, 128, 64, true);
    oled.drawString(0, 56,
        std::format("frame {:3d}", frame), true);
    oled.display();
    std::this_thread::sleep_for(40ms);
  }

  // ── Blank & done ──────────────────────────────────────────────────────────
  oled.clear();
  oled.display();
  oled.setSleep(true);
  std::printf("Done.\n");
  return 0;
}
