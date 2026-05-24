#pragma once

// =============================================================================
//  ui_renderer.h  –  OLED UI state machine + per-screen renderers
//
//  Screens:
//    0 – Joint State   (pos / temp per joint, scrollable)
//    1 – IMU           (gyro + accel + tilt dot, scrollable)
//    2 – Joystick      (stick dots + ABXY/LB/RB buttons)
//    3 – AimRT Log     (scrolling ring-buffer)
//    4 – SBC Status    (CPU cores, temp, RAM bar)
// =============================================================================

#include <array>
#include <chrono>
#include <string_view>

#include "oled_module/event_queue.h"
#include "oled_module/shared_data.h"
#include "oled_driver.hpp"

namespace mybipedal_deploy::oled_module {

// ---------------------------------------------------------------------------
//  UIState  –  top-level display states
// ---------------------------------------------------------------------------
enum class UIState { SLEEPING, MAIN, MENU, SCREEN };

inline constexpr std::array<const char*, 5> kMenuItems = {
    "1 Joint State",
    "2 IMU State",
    "3 Joystick",
    "4 AimRT Log",
    "5 SBC Status",
};

// ---------------------------------------------------------------------------
//  UIContext  –  mutable UI state carried across render ticks
// ---------------------------------------------------------------------------
struct UIContext {
  UIState state       = UIState::SLEEPING;
  int     menu_sel    = 0;  ///< cursor in MENU  [0, 4]
  int     screen_idx  = 0;  ///< active screen   [0, 4]
  int     scroll_offset = 0;

  std::chrono::steady_clock::time_point last_active =
      std::chrono::steady_clock::now();

  void touch() { last_active = std::chrono::steady_clock::now(); }

  [[nodiscard]] bool isTimedOut(std::chrono::seconds timeout) const {
    return std::chrono::steady_clock::now() - last_active >= timeout;
  }
};

// ---------------------------------------------------------------------------
//  Free functions used by OledModule::MainLoop
// ---------------------------------------------------------------------------

/// Update UIContext in response to a single input event.
void handleEvent(Event ev, UIContext& ctx, OledDriver& oled);

/// Render the full frame for the current UIContext + data snapshot.
void renderUI(OledDriver& d, UIContext& ctx, const DataSnapshot& snap);

}  // namespace mybipedal_deploy::oled_module
