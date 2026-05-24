# Mybipedal — Robotics Middleware Engineer Roadmap
### Target: Entry-Level Hire at ~VinMotion (C++ / ROS2 / System Integration)
### Timeline: 8 Weeks | Daily commitment: 2–6 hours

---

## AGENT ADVISORY BOARD

| Agent | Verdict | Key Concern |
|---|---|---|
| **C++ Engineer** | 45% | No RAII, no move semantics, no exception safety |
| **ROS2/Integration Specialist** | 42% | No lifecycle nodes, no ros2_control, no real-time timing |
| **Hiring Manager** | 55% | No unit tests (disqualifier), strong control theory |

**Composite Readiness: ~47%**

---

## PART 1 — SKILL ASSESSMENT

| Skill Area | Score | Status |
|---|---|---|
| C++ Syntax & Compilation | 85% | Leverage |
| Control Theory (PD, RL, gravity projection) | 90% | Leverage |
| Threading (shared_mutex pattern) | 75% | Leverage |
| Eigen / Linear Algebra | 80% | Leverage |
| Module Architecture (AimRT) | 70% | Leverage |
| CAN Bus + Actuator Interface | 40% | Gap — build |
| ROS2 Ecosystem (nodes, launch, lifecycle) | 20% | Gap — urgent |
| Real-Time Linux (mlockall, timerfd, PREEMPT_RT) | 15% | Gap — critical |
| Unit Testing (GTest/GMock) | 5% | **Disqualifier** |
| CI/CD + Docker | 10% | Gap — build |
| ros2_control / hardware_interface | 5% | Gap — build |
| Safety Architecture (E-Stop, watchdog) | 10% | Gap — critical |
| Embedded / Cross-Compilation | 5% | Gap — build |

---

## PART 2 — GAP ANALYSIS vs. JOB REQUIREMENTS

### 🚨 CRITICAL Gaps (Must close before first interview)

**1. ZERO Unit Tests** — Hiring managers at robotics companies will reject on this alone.
Root cause: `build.sh` has `MYBIPEDAL_DEPLOY_BUILD_TESTS=OFF`, no test files exist, no test infrastructure.

**2. No ROS2 Launch File System** — Every real robotics job uses `ros2 launch`.
You have raw YAML configs but no orchestration. Hiring managers assume you don't know the ROS2 ecosystem.

**3. Real-time timing is decorative, not real.** `rt_priority: 80` + `bind_cpu: 3` in YAML but `sleep_until()` in C++.
Not real-time safe. On loaded system, 1ms jitter is guaranteed.

**4. No safety architecture.** No E-Stop, no watchdog, no stale-data guard.
For a bipedal robot, this is a physical safety issue. Companies won't deploy your code on hardware without it.

**5. No ros2_control experience.** You reimplemented what ros2_controllers solves.
Every production humanoid (Unitree, Fourier, etc.) uses ros2_control. If the job mentions hardware integration, you need this.

### ⚠️ HIGH Gaps (Close in weeks 1–4)

- No `rclcpp::Lifecycle` nodes — state machine is custom, which is fine, but you should know the standard approach too
- No ROS2 Actions (actionlib2) — would be relevant for `/teleop`, `/walk_to_goal`
- No ROS2 dynamic parameters (`rclcpp::Parameter`)
- No diagnostic infrastructure (`diagnostic_msgs`, heartbeat monitoring)
- No Docker / containerization — build environment not reproducible
- No CI/CD pipeline — nothing enforces code quality

### 📈 MEDIUM Gaps (Weeks 5–8)

- No cross-compilation (ARM, embedded targets)
- No Linux real-time kernel (PREEMPT_RT) experience
- No embedded RTOS experience
- No performance profiling (latency measurement, CPU flame graphs)

---

## PART 3 — 8-WEEK ROADMAP

| Week | Theme | Focus | Target Skill |
|---|---|---|---|
| **Week 1** | Testing Foundation | Add GTest, test CI/CD pipeline | Testing: 0→40% |
| **Week 2** | ROS2 Core Competency | rclcpp nodes, launch files, lifecycle | ROS2: 20→55% |
| **Week 3** | Production Hardening | Exception safety, memory safety, sanitizers | C++: 45→65% |
| **Week 4** | Real-Time Linux | mlockall, timerfd, thread affinity | RT Linux: 15→50% |
| **Week 5** | ros2_control | hardware_interface, controller_manager | HW Abstraction: 5→40% |
| **Week 6** | Safety Architecture | E-Stop, watchdog, diagnostics | Safety: 10→50% |
| **Week 7** | Docker + CI/CD | Dockerfile, GitHub Actions, test automation | DevOps: 10→50% |
| **Week 8** | Integration + Interview Prep | End-to-end demo, portfolio polish, mock interviews | All: composite +20% |

---

## PART 4 — WEEK 1 DETAIL (Daily Tasks)

> **Goal: Establish testing infrastructure and CI/CD pipeline.**
> This is the highest-leverage investment you can make right now. It closes the #1 disqualifier and signals engineering maturity.

### Day 1 — Testing Infrastructure Setup (2–3 hrs)
```
TODAY:
□ Enable MYBIPEDAL_DEPLOY_BUILD_TESTS=ON in build.sh
□ Create first test file: src/module/control_module/test/throttler_test.cc
  - Test Throttler fires once per interval
  - Test Throttler blocks correctly when called too frequently
□ Create src/module/control_module/test/digital_lp_filter_test.cc
  - Test filter output vs expected IIR response at known frequencies
  - Test init() sets state correctly
□ Run: cmake --build build --target control_module_test
□ Verify tests pass locally
```

**Why this first:** `Throttler` and `digital_lp_filter` are pure math — easy to test, no hardware, no ONNX. Success builds momentum.

### Day 2 — State Machine Tests + GitHub Actions (3–4 hrs)
```
TODAY:
□ Create src/module/control_module/test/state_machine_test.cc
  - Test idle→zero allowed, idle→walk REJECTED (pre-state guard)
  - Test throttler blocks rapid transitions
  - Test RestartController resets state
□ Create .github/workflows/test.yml (GitHub Actions)
  - Build with GCC 13 + tests enabled
  - Run: ctest --output-on-failure
  - Matrix: Ubuntu 22.04
□ Push to branch, verify CI green
□ Open PR, get first CI review
```

**Why GitHub Actions first:** It's free, it's visible to recruiters, and a passing CI pipeline is the single most convincing proof of engineering discipline at your level.

### Day 3 — RL Controller Tests (3–4 hrs)
```
TODAY:
□ Create src/module/control_module/test/rl_controller_test.cc
  - Mock ONNX Runtime session (use Ort::Session mock or interface-based approach)
  - Test observation clipping: values > clip → clamped
  - Test action clipping: same
  - Test is_first_frame_ zero-fills history correctly
  - Test gravity projection (known quaternion → known output)
□ Add Eigen3 to test link libraries in CMakeLists
□ Verify all RL controller tests pass
□ Commit with meaningful message: "Add RLController unit tests (throttler, filter, obs clipping)"
```

**Key insight:** The RL controller math is the most impressive part of your codebase. If you can write tests for the observation buffer and gravity projection, you prove you understand it deeply — which is exactly what interview questions probe.

### Day 4 — Joint Driver + Sim Module Tests (3–4 hrs)
```
TODAY:
□ Create src/module/joint_driver_module/test/transmission_test.cc
  - Test TransimissionManager forward Kinematics (known input → expected output)
  - Test inverse kinematics
□ Create src/module/sim_module/test/sim_module_test.cc
  - Test joint_state_index_map_ initialization
  - Test WriteMotorCmd boundary conditions (empty joint_names_)
□ Run full test suite: ctest --output-on-failure
□ Add test coverage report (lcov/gcov) to GitHub Actions
```

### Day 5 — PD Controller + Integration Test (3–4 hrs)
```
TODAY:
□ Create src/module/control_module/test/pd_controller_test.cc
  - Test transition interpolation: start_joint_angles → init_state over trans_mode_duration
  - Test is_keep_controller holds position
  - Test trans_mode_duration configurable via YAML
□ Create src/module/control_module/test/controller_integration_test.cc
  - Test RLController + PDController can coexist (same joint_names_)
  - Test SetJointStateData with partial joint list (missing joint → gracefully skipped)
□ Add a test summary badge to README.md
□ Write a GitHub commit: "Week 1 complete: test infrastructure, 6 test files, GitHub Actions CI"
```

---

## PART 5 — PROJECT UPGRADE PLAN

### Current State → Hireable Project

Your codebase controls a real bipedal robot. That's already better than most candidates. Here's how to make it hireable.

### Phase 1: Make It Safe to Deploy (Week 1–2)
**Add before ANY interview:**
- [ ] Unit tests for all control algorithms (filter, throttler, state machine, RL obs)
- [ ] NaN check before publishing `joint_cmd`: `if (!std::isfinite(action)) { enter_safe_mode(); }`
- [ ] Watchdog: if `loop_count_` doesn't advance for 5ms, cut motor torque
- [ ] Stale IMU guard: if `imu_data_` age > 50ms, fall back to PD-only mode

### Phase 2: Add ROS2 Standard Integration (Week 2–3)
**This is what interviewers expect to see:**
- [ ] `ros2 launch` file for hardware bringup (`launch/robot_bringup.launch.py`)
- [ ] `ros2 control` integration — replace custom `JointDriverModule` with `ros2_controllers`
  - Use `joint_state_broadcaster`, `position_controllers`
  - Your custom transmission logic becomes a `transmission_interface` plugin
- [ ] `rclcpp_lifecycle` node for the state machine (optional but shows ecosystem knowledge)
- [ ] Dynamic parameter server for Kp/Kd tuning at runtime
- [ ] `diagnostic_msgs` publisher: actuator heartbeat, IMU rate, loop timing jitter

### Phase 3: Real-Time Hardening (Week 4–5)
**This separates you from other candidates:**
- [ ] Replace `sleep_until()` with `timerfd_create(CLOCK_MONOTONIC)` + `timerfd_settime()`
- [ ] Add `mlockall(MCL_CURRENT | MCL_FUTURE)` to main loop thread
- [ ] Measure and log actual loop jitter: `log_jitter = now - next_iteration_time`
- [ ] Add pthread `SCHED_FIFO` + `sched_setscheduler()` call in module init
- [ ] Document: "Achieved <X>us median jitter, <Y>us worst-case on stock Ubuntu 22.04"

### Phase 4: Docker + CI/CD (Week 6–7)
**Required for any company that cares about deployment:**
- [ ] `Dockerfile` — GCC 13, ROS 2 Humble, MuJoCo, ONNX Runtime, build deps
- [ ] `docker-compose.yml` for sim + hardware mode
- [ ] GitHub Actions: build → test → lint → clang-tidy → performance benchmark
- [ ] Add `clang-tidy` checks to CI (fix every warning it finds)
- [ ] Add AArch64 cross-compile target to CMake (shows embedded awareness)

### Phase 5: Portfolio Polish (Week 8)
**What you send to recruiters:**
- [ ] README.md with: architecture diagram, how to build, how to run, control loop diagram
- [ ] Video of sim (GLFW window recording) + real robot walk
- [ ] Benchmark: inference latency (ONNX), control loop jitter, actuator tracking error
- [ ] Blog post: "How I built a 1kHz RL whole-body controller for a bipedal robot"
- [ ] GitHub repo with clean commit history + PR descriptions

---

## PART 6 — TOOLING + FOCUS SYSTEM

### Task Tracking
```
Use: GitHub Projects (per-repo, free) + CLI Todoist integration
Structure:
  - Backlog (all 8-week tasks, sorted by priority)
  - This Week (max 7 tasks)
  - Today (max 3 tasks)
  - Done (logged daily)

Daily workflow:
  06:00 — Review GitHub Projects, set Today (5 min)
  06:05 — Deep work block 1 (2–3 hrs, no phone, no Slack)
  09:00 — Break + review + task update (15 min)
  09:15 — Deep work block 2 (2–3 hrs)
  11:30 — Daily commit + GitHub Actions check
  12:00 — Weekly review (every Friday)
```

### Progress Tracking (Weekly Metrics)
```
Every Friday, update this sheet:

C++ Proficiency:     __% (target: 65% by week 4)
ROS2 Knowledge:      __% (target: 55% by week 3)
Testing Coverage:    __% (target: 60% by week 2)
RT Linux Skills:     __% (target: 50% by week 5)
DevOps Score:        __% (target: 50% by week 7)
Safety Architecture: __% (target: 50% by week 6)

Tests Written This Week: ___
CI/CD Green Checks: ___
Interview Questions Practiced: ___
New Concepts Learned: ___
```

### CLI Progress Tracker
```bash
# Add to ~/.bashrc
alias today="echo '=== TODAY ===' && git log --oneline -3 && echo '---' && cat ~/progress.md"
alias log-skill="echo '$(date +%Y-%m-%d): $1' >> ~/skill_log.md"

# Usage:
$ log-skill "Week 1 Day 1: testing infrastructure setup complete"
$ today
```

### Focus System (Deep Work)
```
Environment: macOS + iTerm2 + tmux
Setup:
  - Full screen terminal, no other windows
  - iPhone in another room
  - Music: brown noise or instrumental only
  - Pomodoro: 50 min work / 10 min break
  - 3 pomodoros per deep work block = 2.5 hrs productive time

For coding:
  - Vim or CLion (no VS Code, no IDE mouse dependence)
  - tmux sessions: one per module being worked on
  - Fast feedback: rebuild only changed target, not full project
```

---

## PART 7 — REWARD + ACCOUNTABILITY SYSTEM

### EXP / Level System

```
Every significant action earns EXP:
  - Write a passing unit test:          +50 EXP
  - Close a GitHub issue:              +100 EXP
  - Complete a daily task:             +30 EXP
  - Fix a bug found by a test:          +75 EXP
  - Pass a mock interview question:   +100 EXP
  - Complete a week's goals:         +500 EXP
  - Open a PR that merges:            +200 EXP

Level thresholds:
  Level 1  (0–500)      → "Intern"
  Level 2  (501–1500)   → "Junior Engineer"
  Level 3  (1501–4000)  → "Mid Engineer"
  Level 4  (4001–8000)  → "Senior Ready"
  Level 5  (8001+)       → "Hireable"

Track: ~/exp_tracker.md (update daily)
```

### Daily Streak System
```
Streak = consecutive days with ≥ 1 meaningful commit
  - 7-day streak:  +100 EXP bonus
  - 30-day streak: +500 EXP bonus + "consistency is your superpower" message
  - Break streak:   -200 EXP penalty + "recalibrating..." message

Accountability: Share streak on Twitter/LinkedIn/X — public accountability
```

### Rewards (Non-Monetary)
```
Level 2 unlocked:  Get a coffee at your favorite café, work from there
Level 3 unlocked:  Buy that book you've been putting off
Level 4 unlocked:  Record your first mock interview video
Level 5 unlocked:  Apply to 3 real jobs + send recruiter messages

Weekly reward (if all 5 weekly tasks done):
  Friday evening: 2 hrs gaming / show / anything you want
  No guilt. You earned it.
```

### Penalty System
```
If you miss 2 consecutive days:
  → Sunday: written reflection: "What broke? What's the fix?"
  → Share with accountability partner (if you have one)

If you miss a weekly target:
  → Saturday: 4-hour catch-up session (no rewards until caught up)
  → No new topic until backlog is cleared
```

---

## PART 8 — INTERVIEW PREPARATION

### Questions You WILL Be Asked (Based on Codebase Analysis)

**Q1 — Threading Model** *(Every interview)*
> "Walk me through the threading model in ControlModule. Who holds which locks, when, and why? Is the main loop thread-safe?"

You wrote it. Know it cold. Know the `shared_mutex` reader-writer pattern. Know that callbacks use `shared_lock` and the main loop reads without locking (which is actually OK if the reader and writer are the same thread, but verify this).

**Q2 — Observation Buffer** *(Every interview)*
> "The RL policy runs at 250Hz (decimation=4 at 1kHz). Walk me through the observation buffer. How does `propri_history_buffer` shift? What does `is_first_frame_` do and why?"

This is your showcase answer. You built this. Explain: shift register, history initialization to zero (so policy sees "still" robot initially), observation clipping as safety margin.

**Q3 — Gravity Projection** *(Controls-focused interview)*
> "What does `GetRotationMatrixFromZyxEulerAngles(QuatToZyx(quat)).inverse() * gravity_vector` do, and why?"

You wrote this. Explain: quaternions are singularity-free, ZYX converts to euler angles, rotation matrix R^T * g_world projects gravity into robot body frame, this tells the policy which way is "down" regardless of orientation.

**Q4 — Real-Time Safety** *(Senior interview)*
> "Your control loop runs at 1kHz with `sleep_until()`. Is this real-time safe? What would you change?"

This is the question that will expose the gap. Your YAML has `rt_priority: 80` but your C++ uses `sleep_until()`. Be honest: "I configured real-time priority in YAML but the actual timing is best-effort. The fix is `timerfd_settime(CLOCK_MONOTONIC)` and `mlockall()`."

**Q5 — Testing** *(Every interview)*
> "You have no unit tests in this codebase. What would you test, and what would your mock setup look like?"

Don't be defensive. Say: "You're right. Here's what I'd test first: the digital low-pass filter (known frequency response), the state machine (invalid transitions), and the RL observation clipping (edge cases). I'd use GTest with mock ROS subscribers and Eigen test utilities."

**Q6 — ROS2 Gap** *(Every interview at ROS2 companies)*
> "Why did you use AimRT instead of rclcpp? Have you used ros2_control?"

Honest answer: "I used AimRT because it provides ROS2 compatibility as a plugin while being lighter weight for embedded deployment. I'm currently learning ros2_control — it's on my roadmap for Week 5." *(This is what the roadmap is for.)*

---

## PART 9 — IMMEDIATE NEXT ACTIONS (Do Today)

```
RIGHT NOW:
□ Open /home/long/LongVH34/Mybipedal-deploy/build.sh
  → Change MYBIPEDAL_DEPLOY_BUILD_TESTS=OFF → ON
□ mkdir -p src/module/control_module/test/
□ Create src/module/control_module/test/throttler_test.cc (copy the Throttler logic, test it)
□ Run: ./build.sh && ctest
□ If tests pass: commit as "test: add testing infrastructure skeleton"

THIS WEEK (before next session):
□ Complete all Week 1 tasks (Days 1–5 above)
□ Set up GitHub Actions CI pipeline
□ Add NaN check to RLController::GetJointCmdData()
□ Add stale-IMU guard to ControlModule MainLoop
□ Open GitHub issue: "Add watchdog for control loop timing"
□ Update CLAUDE.md with new testing commands
```

---

## THE ONE-LINE SUMMARY FOR RECRUITERS

> "I built a 1kHz whole-body controller for a bipedal robot using ONNX RL policy inference + PD tracking, with MuJoCo simulation and real CAN/Robstride hardware support, organized as AimRT plugins in C++20 with YAML-driven configuration."

**That sentence gets callbacks. Everything in this document makes it true.**
