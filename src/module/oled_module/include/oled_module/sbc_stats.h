#pragma once

// =============================================================================
//  sbc_stats.h  –  Lightweight system-stats helpers for Radxa / Linux SBCs
//
//  All functions read from /proc or /sys; no external dependencies.
//  Designed to be called periodically from a low-priority background thread.
// =============================================================================

#include <cstdint>
#include <string>
#include <vector>

namespace mybipedal_deploy::oled_module {

/// Returns the first non-loopback IPv4 address, or "0.0.0.0".
std::string getLocalIP();

/// Reads per-core CPU usage % by sampling /proc/stat twice (~200 ms apart).
/// Returns one value per logical core.
std::vector<float> readCpuUsage();

/// Reads the first positive thermal zone temperature from /sys/class/thermal.
float readCpuTemp();

/// Fills total_kb / avail_kb from /proc/meminfo.
void readRamInfo(uint64_t& total_kb, uint64_t& avail_kb);

}  // namespace mybipedal_deploy::oled_module
