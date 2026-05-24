#pragma once

// =============================================================================
//  event_queue.h  –  Thread-safe event queue for UI input events
//
//  Produced by: gpioThread  (encoder rotations + button presses)
//  Consumed by: OledModule::MainLoop  (UI state machine)
// =============================================================================

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <queue>

namespace mybipedal_deploy::oled_module {

// ---------------------------------------------------------------------------
//  UI input events
// ---------------------------------------------------------------------------
enum class Event {
  ENC_CW,    ///< Encoder rotated clockwise     (scroll down / next item)
  ENC_CCW,   ///< Encoder rotated counter-CW    (scroll up  / prev item)
  ENC_PUSH,  ///< Encoder button pressed         (confirm / enter)
  BTN_BACK,  ///< Dedicated back button pressed  (cancel / return)
};

// ---------------------------------------------------------------------------
//  EventQueue  –  MPSC (gpio thread → UI loop)
// ---------------------------------------------------------------------------
class EventQueue {
 public:
  void push(Event e) {
    {
      std::lock_guard lock(mtx_);
      q_.push(e);
    }
    cv_.notify_one();
  }

  /// Block until an event arrives or timeout elapses.
  /// Returns std::nullopt on timeout.
  std::optional<Event> pop(std::chrono::milliseconds timeout) {
    std::unique_lock lock(mtx_);
    if (!cv_.wait_for(lock, timeout, [&] { return !q_.empty(); }))
      return std::nullopt;
    Event e = q_.front();
    q_.pop();
    return e;
  }

  /// Discard all pending events (used after waking from sleep to swallow
  /// mechanical bounce events generated during the first keypress).
  void clear() {
    std::lock_guard lock(mtx_);
    std::queue<Event> empty;
    std::swap(q_, empty);
  }

 private:
  std::queue<Event>       q_;
  std::mutex              mtx_;
  std::condition_variable cv_;
};

}  // namespace mybipedal_deploy::oled_module
