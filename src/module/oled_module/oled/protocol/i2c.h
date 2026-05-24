#pragma once

#include <cstdint>
#include <initializer_list>
#include <span>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
//  I2CDevice  –  thin RAII wrapper around a Linux I²C-dev file descriptor
//
//  Usage:
//    auto dev = std::make_unique<I2CDevice>("/dev/i2c-7", 0x3C);
//    if (!dev->open()) { /* handle error */ }
//    dev->write({0x00, 0xAF});   // send command byte + data
// ---------------------------------------------------------------------------
class I2CDevice {
 public:
  I2CDevice(std::string device, uint8_t addr);
  ~I2CDevice();

  // Non-copyable, movable
  I2CDevice(const I2CDevice&)            = delete;
  I2CDevice& operator=(const I2CDevice&) = delete;
  I2CDevice(I2CDevice&&) noexcept;
  I2CDevice& operator=(I2CDevice&&) noexcept;

  bool open();
  void close();
  [[nodiscard]] bool isOpen() const { return fd_ >= 0; }

  /// Write a raw byte buffer to the I²C slave.
  bool write(std::span<const uint8_t> data);
  bool write(std::initializer_list<uint8_t> data);
  bool write(const std::vector<uint8_t>& data);

 private:
  std::string device_;
  uint8_t     addr_{0};
  int         fd_{-1};
};
