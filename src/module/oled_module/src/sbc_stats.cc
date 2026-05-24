#include "oled_module/sbc_stats.h"

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <format>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

namespace mybipedal_deploy::oled_module {

// ---------------------------------------------------------------------------
std::string getLocalIP() {
  ifaddrs* ifa = nullptr;
  if (::getifaddrs(&ifa) < 0) return "?.?.?.?";
  std::string result = "0.0.0.0";
  for (auto* p = ifa; p; p = p->ifa_next) {
    if (!p->ifa_addr || p->ifa_addr->sa_family != AF_INET) continue;
    if (std::string(p->ifa_name) == "lo") continue;
    char buf[INET_ADDRSTRLEN];
    auto* sa = reinterpret_cast<sockaddr_in*>(p->ifa_addr);
    ::inet_ntop(AF_INET, &sa->sin_addr, buf, sizeof(buf));
    result = buf;
    break;
  }
  ::freeifaddrs(ifa);
  return result;
}

// ---------------------------------------------------------------------------
std::vector<float> readCpuUsage() {
  // Read /proc/stat twice, 200 ms apart, then compute per-core usage.
  auto readStats = []() -> std::vector<std::array<uint64_t, 2>> {
    std::ifstream f("/proc/stat");
    std::vector<std::array<uint64_t, 2>> v;
    std::string line;
    while (std::getline(f, line)) {
      if (line.rfind("cpu", 0) != 0 || line[3] == ' ') continue;
      std::istringstream ss(line.substr(5));
      uint64_t user, nice, sys, idle, iowait, irq, softirq;
      ss >> user >> nice >> sys >> idle >> iowait >> irq >> softirq;
      uint64_t total = user + nice + sys + idle + iowait + irq + softirq;
      v.push_back({total, idle + iowait});
    }
    return v;
  };

  auto s1 = readStats();
  std::this_thread::sleep_for(200ms);
  auto s2 = readStats();

  std::vector<float> pct;
  for (size_t i = 0; i < std::min(s1.size(), s2.size()); ++i) {
    uint64_t dt    = s2[i][0] - s1[i][0];
    uint64_t didle = s2[i][1] - s1[i][1];
    pct.push_back(dt == 0 ? 0.f : 100.f * (1.f - static_cast<float>(didle) / dt));
  }
  return pct;
}

// ---------------------------------------------------------------------------
float readCpuTemp() {
  for (int z = 0; z < 6; ++z) {
    std::ifstream f(std::format("/sys/class/thermal/thermal_zone{}/temp", z));
    if (!f) continue;
    int millideg;
    f >> millideg;
    if (millideg > 0) return millideg / 1000.f;
  }
  return 0.f;
}

// ---------------------------------------------------------------------------
void readRamInfo(uint64_t& total_kb, uint64_t& avail_kb) {
  std::ifstream f("/proc/meminfo");
  std::string key, unit;
  uint64_t val;
  total_kb = avail_kb = 0;
  while (f >> key >> val >> unit) {
    if (key == "MemTotal:")     total_kb = val;
    if (key == "MemAvailable:") avail_kb = val;
  }
}

}  // namespace mybipedal_deploy::oled_module
