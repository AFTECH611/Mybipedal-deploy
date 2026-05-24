#include "i2c.h"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <stdexcept>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

// ---------------------------------------------------------------------------
I2CDevice::I2CDevice(std::string device, uint8_t addr)
    : device_(std::move(device)), addr_(addr) {}

I2CDevice::~I2CDevice() { close(); }

I2CDevice::I2CDevice(I2CDevice&& o) noexcept
    : device_(std::move(o.device_)), addr_(o.addr_), fd_(o.fd_) {
  o.fd_ = -1;
}

I2CDevice& I2CDevice::operator=(I2CDevice&& o) noexcept {
  if (this != &o) {
    close();
    device_ = std::move(o.device_);
    addr_   = o.addr_;
    fd_     = o.fd_;
    o.fd_   = -1;
  }
  return *this;
}

// ---------------------------------------------------------------------------
bool I2CDevice::open() {
  fd_ = ::open(device_.c_str(), O_RDWR);
  if (fd_ < 0) {
    std::perror(("I2CDevice::open " + device_).c_str());
    return false;
  }
  if (::ioctl(fd_, I2C_SLAVE, static_cast<long>(addr_)) < 0) {
    std::perror("I2CDevice::open I2C_SLAVE");
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  return true;
}

void I2CDevice::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool I2CDevice::write(std::span<const uint8_t> data) {
  if (fd_ < 0) return false;
  ssize_t n = ::write(fd_, data.data(), data.size());
  return n == static_cast<ssize_t>(data.size());
}

bool I2CDevice::write(std::initializer_list<uint8_t> data) {
  std::vector<uint8_t> buf(data);
  return write(std::span<const uint8_t>{buf.data(), buf.size()});
}

bool I2CDevice::write(const std::vector<uint8_t>& data) {
  return write(std::span<const uint8_t>{data.data(), data.size()});
}
