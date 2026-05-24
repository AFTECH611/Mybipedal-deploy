#include "ui_renderer.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <format>
#include <string>

namespace mybipedal_deploy::oled_module {

// ============================================================================
//  Internal helpers
// ============================================================================
static void drawTitle(OledDriver& d, std::string_view title) {
  d.fillRect(0, 0, 128, 10, true);
  int tx = (128 - d.textWidth(title)) / 2;
  d.drawString(tx, 1, title, false);
  d.drawHLine(0, 10, 128, true);
}

static void drawScrollbar(OledDriver& d, int scroll, int content_h) {
  constexpr int VIEW_H = 54, TRACK_Y = 10;
  if (content_h <= VIEW_H) return;
  int max_s = content_h - VIEW_H;
  int bar_h = std::max(6, VIEW_H * VIEW_H / content_h);
  int bar_y = TRACK_Y + scroll * (VIEW_H - bar_h) / max_s;
  d.fillRect(126, bar_y, 2, bar_h, true);
}

// ============================================================================
//  Screen: MAIN
// ============================================================================
static void renderMain(OledDriver& d, const DataSnapshot& sd) {
  const std::string line1 = "MyBipedal";
  const std::string line2 = "By LongVu";
  int x1 = (128 - d.textWidth(line1, 2)) / 2;
  int x2 = (128 - d.textWidth(line2))    / 2;
  d.drawString(x1, 4,  line1, true, 2);
  d.drawString(x2, 24, line2, true, 1);
  d.drawHLine(0, 35, 128, true);
  d.drawString(0, 38, "IP:", true);
  d.drawString(18, 38, sd.robot_ip, true);
  d.drawString(0, 54, "  Rotate enc -> Menu  ", true);
}

// ============================================================================
//  Screen: MENU
// ============================================================================
static void renderMenu(OledDriver& d, int sel) {
  drawTitle(d, "-- MENU --");
  for (int i = 0; i < 5; ++i) {
    int y = 12 + i * 10;
    if (i == sel) {
      d.fillRect(0, y, 128, 10, true);
      d.drawString(2, y + 1, kMenuItems[i], false);
    } else {
      d.drawString(2, y + 1, kMenuItems[i], true);
    }
  }
}

// ============================================================================
//  Screen 0: Joint State
// ============================================================================
static void renderJointState(OledDriver& d, const DataSnapshot& sd, int& scroll) {
  drawTitle(d, "Joint State");
  const auto& J = sd.joints;
  int content_h = ((int(J.size()) + 1) / 2) * 17;
  scroll = std::clamp(scroll, 0, std::max(0, content_h - 54));

  for (int i = 0; i < (int)J.size() && i < 12; ++i) {
    int x = (i % 2) * 64;
    int y = 13 + (i / 2) * 17 - scroll;
    if (y + 16 < 10 || y > 63) continue;
    d.drawString(x,    y,   J[i].name, true);
    d.drawString(x,    y+8, std::format("{:.1f}", J[i].pos),   true);
    d.drawString(x+36, y+8, std::format("{:.0f}C", J[i].temp), true);
  }
  drawScrollbar(d, scroll, content_h);
}

// ============================================================================
//  Screen 1: IMU
// ============================================================================
static void renderIMU(OledDriver& d, const DataSnapshot& sd, int& scroll) {
  drawTitle(d, "IMU State");
  constexpr int CONTENT_H = 60;
  scroll = std::clamp(scroll, 0, std::max(0, CONTENT_H - 54));

  const std::pair<int, std::string> rows[] = {
      {12, std::format("Gx{:+6.1f}", sd.imu_gx)},
      {22, std::format("Gy{:+6.1f}", sd.imu_gy)},
      {32, std::format("Gz{:+6.1f}", sd.imu_gz)},
      {42, std::format("Ax{:+5.2f}", sd.imu_ax)},
      {52, std::format("Ay{:+5.2f}", sd.imu_ay)},
      {62, std::format("Az{:+5.2f}", sd.imu_az)},
  };
  for (auto& [y, text] : rows) {
    int sy = y - scroll;
    if (sy >= 10 && sy < 64) d.drawString(0, sy, text, true);
  }

  // Tilt dot (fixed position, no scroll)
  constexpr int CX=100, CY=38, R=22;
  d.drawCircle(CX, CY, R, true);
  d.drawHLine(CX-R, CY, 2*R+1, true);
  d.drawVLine(CX, CY-R, 2*R+1, true);
  float nx = std::clamp(sd.imu_gy / 90.f, -1.f, 1.f);
  float ny = std::clamp(sd.imu_gx / 90.f, -1.f, 1.f);
  d.fillCircle(CX + (int)(nx*(R-3)), CY + (int)(ny*(R-3)), 3, true);
  drawScrollbar(d, scroll, CONTENT_H);
}

// ============================================================================
//  Screen 2: Joystick
// ============================================================================
static void renderJoystick(OledDriver& d, const DataSnapshot& sd) {
  drawTitle(d, "Joystick");
  auto btn     = [&](int bit) { return (sd.joy_buttons >> bit) & 1u; };
  auto drawBtn = [&](int x, int y, const char* label, bool pressed) {
    int w = d.textWidth(label) + 2;
    if (pressed) {
      d.fillRect(x-1, y-1, w, 9, true);
      d.drawString(x, y, label, false);
    } else {
      d.drawString(x, y, label, true);
    }
  };

  // Top row: LB | Bk | St | RB
  drawBtn(1,   12, "LB", btn(4));
  drawBtn(43,  12, "Bk", btn(7));
  drawBtn(69,  12, "St", btn(6));
  drawBtn(114, 12, "RB", btn(5));

  // Left stick (larger, dominant)
  constexpr int LX=22, LY=32, LR=14;
  d.drawCircle(LX, LY, LR, true);
  d.drawHLine(LX-LR, LY, 2*LR+1, true);
  d.drawVLine(LX, LY-LR, 2*LR+1, true);
  d.fillCircle(LX + (int)(sd.joy_lx*(LR-3)),
               LY + (int)(sd.joy_ly*(LR-3)), 3, true);

  // Right stick (smaller)
  constexpr int RX=82, RY=22, RR=8;
  d.drawCircle(RX, RY, RR, true);
  d.drawHLine(RX-RR, RY, 2*RR+1, true);
  d.drawVLine(RX, RY-RR, 2*RR+1, true);
  d.fillCircle(RX + (int)(sd.joy_rx*(RR-2)),
               RY + (int)(sd.joy_ry*(RR-2)), 2, true);

  // ABXY diamond
  drawBtn(107, 28, "Y", btn(3));
  drawBtn(96,  37, "X", btn(2));
  drawBtn(116, 37, "B", btn(1));
  drawBtn(107, 46, "A", btn(0));

  // Analog values
  d.drawString(0,  56, std::format("L{:+.1f} {:+.1f}", sd.joy_lx, sd.joy_ly), true);
  d.drawString(66, 56, std::format("R{:+.1f} {:+.1f}", sd.joy_rx, sd.joy_ry), true);
}

// ============================================================================
//  Screen 3: AimRT Log
// ============================================================================
static void renderLog(OledDriver& d, const DataSnapshot& sd, int& scroll) {
  drawTitle(d, "AimRT Log");
  constexpr int kRowH = 9, kVisRows = 6, kChars = 21;
  const auto& lines = sd.log_lines;
  int total      = (int)lines.size();
  int max_scroll = std::max(0, total - kVisRows);
  scroll = std::clamp(scroll, 0, max_scroll);

  int start = std::max(0, total - kVisRows - scroll);
  for (int i = 0; i < kVisRows && (start+i) < total; ++i) {
    std::string row = lines[start+i];
    if ((int)row.size() > kChars) row.resize(kChars);
    d.drawString(0, 12 + i*kRowH, row, true);
  }
  if (total > kVisRows) {
    int bar_h = std::max(6, kVisRows * 54 / total);
    int bar_y = 10 + (max_scroll > 0 ? scroll * (54 - bar_h) / max_scroll : 0);
    d.fillRect(126, bar_y, 2, bar_h, true);
  }
}

// ============================================================================
//  Screen 4: SBC Status
// ============================================================================
static void renderSBC(OledDriver& d, const DataSnapshot& sd, int& scroll) {
  drawTitle(d, "SBC Status");
  constexpr int CONTENT_H = 58;
  scroll = std::clamp(scroll, 0, std::max(0, CONTENT_H - 54));

  const int ncores = (int)sd.cpu_core_pct.size();
  for (int i = 0; i < ncores && i < 8; ++i) {
    int x = (i / 4) * 64;
    int y = 13 + (i % 4) * 8 - scroll;
    if (y < 10 || y > 63) continue;
    char lbl[4]; std::snprintf(lbl, sizeof(lbl), "C%d", i);
    d.drawString(x, y, lbl, true);
    d.drawBar(x+12, y, 48, 5, (int)sd.cpu_core_pct[i], true);
  }
  int y_bot = 45 - scroll;
  if (y_bot >= 10 && y_bot < 64) {
    d.drawString(0, y_bot, std::format("T:{:.0f}C", sd.cpu_temp_c), true);
    uint64_t used_kb = sd.ram_total_kb > sd.ram_avail_kb
                     ? sd.ram_total_kb - sd.ram_avail_kb : 0;
    d.drawString(48, y_bot,
        std::format("RAM{:.1f}/{:.0f}G",
            used_kb / (1024.f*1024.f),
            sd.ram_total_kb / (1024.f*1024.f)), true);
  }
  if (y_bot + 9 >= 10 && y_bot + 9 < 64) {
    int pct = sd.ram_total_kb
            ? (int)(100.f * (sd.ram_total_kb - sd.ram_avail_kb) / sd.ram_total_kb) : 0;
    d.drawBar(0, y_bot+9, 128, 5, pct, true);
  }
  drawScrollbar(d, scroll, CONTENT_H);
}

// ============================================================================
//  Public: renderUI
// ============================================================================
void renderUI(OledDriver& d, UIContext& ctx, const DataSnapshot& snap) {
  d.clear();
  if      (ctx.state == UIState::SLEEPING) { /* blank */ }
  else if (ctx.state == UIState::MAIN)     renderMain(d, snap);
  else if (ctx.state == UIState::MENU)     renderMenu(d, ctx.menu_sel);
  else {
    switch (ctx.screen_idx) {
      case 0: renderJointState(d, snap, ctx.scroll_offset); break;
      case 1: renderIMU       (d, snap, ctx.scroll_offset); break;
      case 2: renderJoystick  (d, snap);                    break;
      case 3: renderLog       (d, snap, ctx.scroll_offset); break;
      case 4: renderSBC       (d, snap, ctx.scroll_offset); break;
    }
  }
  d.display();
}

// ============================================================================
//  Public: handleEvent  –  UI state machine transitions
// ============================================================================
void handleEvent(Event ev, UIContext& ctx, OledDriver& oled) {
  ctx.touch();

  if (ctx.state == UIState::SLEEPING) {
    ctx.state = UIState::MAIN;
    oled.setSleep(false);
    return;
  }

  switch (ctx.state) {
    // ── MAIN ──────────────────────────────────────────────────────────────
    case UIState::MAIN:
      if (ev == Event::ENC_CW || ev == Event::ENC_CCW || ev == Event::ENC_PUSH)
        ctx.state = UIState::MENU;
      // BTN_BACK: already at top level, nothing to do
      break;

    // ── MENU ──────────────────────────────────────────────────────────────
    case UIState::MENU:
      if (ev == Event::ENC_CW)
        ctx.menu_sel = (ctx.menu_sel + 1) % 5;
      else if (ev == Event::ENC_CCW)
        ctx.menu_sel = (ctx.menu_sel + 4) % 5;
      else if (ev == Event::ENC_PUSH) {
        ctx.screen_idx    = ctx.menu_sel;
        ctx.state         = UIState::SCREEN;
        ctx.scroll_offset = 0;
      } else if (ev == Event::BTN_BACK)
        ctx.state = UIState::MAIN;
      break;

    // ── SCREEN ────────────────────────────────────────────────────────────
    case UIState::SCREEN:
      if (ev == Event::ENC_CW)
        ctx.scroll_offset += 8;  // renderer clamps the max
      else if (ev == Event::ENC_CCW)
        ctx.scroll_offset = std::max(0, ctx.scroll_offset - 8);
      else if (ev == Event::BTN_BACK || ev == Event::ENC_PUSH) {
        ctx.state         = UIState::MENU;
        ctx.scroll_offset = 0;
      }
      break;

    default: break;
  }
}

}  // namespace mybipedal_deploy::oled_module
