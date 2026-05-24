#pragma once

// =============================================================================
//  gpio_input.h  –  Lightweight libgpiod v1 input-pin RAII wrapper
//
//  Used by gpioThread() to read rotary encoder and push-buttons without
//  requiring elevated privileges (uses the chardev ABI).
//
//  Targets: Radxa 5B+ (RK3588), libgpiod v1.x  (gpiod_chip / gpiod_line API)
// =============================================================================

#include <stdexcept>
#include <gpiod.h>

namespace mybipedal_deploy::oled_module {

// ---------------------------------------------------------------------------
//  Hardware pin descriptor  (chip path + line offset)
// ---------------------------------------------------------------------------
struct GpioPin {
  const char*  chip;    ///< e.g. "/dev/gpiochip3"
  unsigned int line;    ///< line offset within the chip
};

// ---------------------------------------------------------------------------
//  InputPin  –  RAII wrapper; opens chip + line in INPUT/pull-up mode.
//
//  Read()  returns 1 (high / not pressed) or 0 (low / pressed) following
//  the active-low convention of the hardware.
//
//  Throws std::runtime_error on construction failure so the caller can
//  catch it once and mark the GPIO thread as degraded.
// ---------------------------------------------------------------------------
class InputPin {
 public:
  explicit InputPin(const GpioPin& pin) {
    chip_ = gpiod_chip_open(pin.chip);
    if (!chip_) throw std::runtime_error("InputPin: cannot open chip");

    line_ = gpiod_chip_get_line(chip_, pin.line);
    if (!line_) {
      gpiod_chip_close(chip_);
      chip_ = nullptr;
      throw std::runtime_error("InputPin: cannot get line");
    }

    gpiod_line_request_config cfg{};
    cfg.consumer     = "oled_module";
    cfg.request_type = GPIOD_LINE_REQUEST_DIRECTION_INPUT;
    cfg.flags        = GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP;

    if (gpiod_line_request(line_, &cfg, 0) < 0) {
      gpiod_chip_close(chip_);
      chip_ = nullptr;
      throw std::runtime_error("InputPin: cannot request input with pull-up");
    }
  }

  ~InputPin() {
    if (line_) gpiod_line_release(line_);
    if (chip_)  gpiod_chip_close(chip_);
  }

  // Non-copyable, non-movable (owns a kernel handle)
  InputPin(const InputPin&)            = delete;
  InputPin& operator=(const InputPin&) = delete;

  /// Returns 0 (LOW/pressed) or 1 (HIGH/released).
  [[nodiscard]] int Read() const {
    return gpiod_line_get_value(line_);
  }

 private:
  gpiod_chip* chip_{nullptr};
  gpiod_line* line_{nullptr};
};

}  // namespace mybipedal_deploy::oled_module
