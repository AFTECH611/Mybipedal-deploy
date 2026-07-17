## Build

```bash
# Build qpOASES thirdparty lib
# Build Onnxruntime thirdparty lib
git clone --recursive -b v1.19.2 https://github.com/microsoft/onnxruntime
cd ~/onnxruntime
sed -i 's/be8be39fdbc6e60e94fa7870b280707069b5b81a/32b145f525a8308d7ab1c09388b2e288312d8eba/' cmake/deps.txt
./build.sh --config Release --build_shared_lib --parallel 2 \
  --skip_tests \
  --cmake_extra_defines CMAKE_POLICY_VERSION_MINIMUM=3.5 onnxruntime_BUILD_UNIT_TESTS=OFF
cd ~/Mybipedal/src/module/control_module/third_party/lib
# Copy bản mới vào
cp ~/onnxruntime/build/Linux/Release/libonnxruntime.so.1.19.2 .
cp ~/onnxruntime/build/Linux/Release/libonnxruntime_providers_shared.so .

ln -sf libonnxruntime.so.1.19.2 libonnxruntime.so.1
ln -sf libonnxruntime.so.1 libonnxruntime.so
# Full build (CMake + compile + install to ./build/)
source /opt/ros/humble/setup.bash
./build.sh

# Custom cmake options:
cmake -B build -DCMAKE_BUILD_TYPE=Release \
  -DMYBIPEDAL_DEPLOY_BUILD_TESTS=ON \
  -DMYBIPEDAL_DEPLOY_SIMULATION=ON
cmake --build build --config Release --target install --parallel "$(nproc)"

# Single module rebuild (after editing one module):
cmake --build build --target pkg1  # rebuilds libpkg1.so
cmake --build build --target install  # copies to build/bin
```

## Run

```bash
# Hardware mode (requires CAN bus + hardware)
cd build/bin
sudo setcap cap_net_raw=ep ./aimrt_main
./aimrt_main --cfg_file_path=./cfg/x1_cfg.yaml
# If program cannot run on Radxa 5b+ when executing ./run.sh try this:
sudo LD_LIBRARY_PATH=/opt/ros/humble/lib ./run.sh

# Simulation mode (MuJoCo + GLFW, no hardware)
./run_sim.sh  # → aimrt_main --cfg_file_path=./cfg/x1_cfg_sim.yaml

# With ROS 2 recording
./run_with_recording.sh
```

## Architecture

**AimRT application** — all 5 modules are AimRT plugins loaded from `libpkg1.so` via `pkg/pkg1/pkg_main.cc`.

### Modules & Data Flow

```
JoyStickModule ──(cmd_vel_limiter)──▶ ControlModule ◀──(imu/data, joint_states)
                                       │
                                       ├─ RLController (ONNX inference @ 250Hz)
                                       │   └─ publishes joint_cmd
                                       ├─ PDController (1 kHz motor tracking)
                                       └─ StateMachine (idle → zero → stand → walk)
                                              ▲
                                              └── triggered by joystick buttons

SimMode:  ControlModule ──(joint_cmd)──▶ SimModule (MuJoCo + GLFW)
           ▲                                       │
           └──◀── (joint_states, imu/data) ────────┘

Hardware: ControlModule ──(joint_cmd)──▶ JointDriverModule (CAN → Robstride actuators)
           ▲                                       │
           └──◀── (joint_states) ──────────────────┘

           ImuModule (serial/921600) ──(imu/data)──▶ ControlModule
```

### Key Classes

| Class | Location | Purpose |
|---|---|---|
| `RLController` | `control_module/src/rl_controller.cc` | Loads ONNX policy, builds 36-dim observation, outputs 8-dim actions |
| `PDController` | `control_module/src/pd_controller.cc` | Tracks reference trajectories with Kp/Kd per state |
| `StateMachine` | `control_module/include/control_module/state_machine.h` | Manages idle/zero/stand/walk transitions |
| `TransimissionManager` | `joint_driver_module/include/transmission.h` | Actuator ↔ joint coordinate transforms |
| `Simulate` | `sim_module/src/simulate.cc` | MuJoCo GLFW viewer, publishes simulated sensor data |

### Config Files

- `src/install/linux/bin/cfg/x1_cfg.yaml` — hardware mode (all modules except SimModule)
- `src/install/linux/bin/cfg/x1_cfg_sim.yaml` — simulation mode (replaces JointDriver + Imu with SimModule)
- Module-specific configs in `*/cfg/*.yaml` (override via `x1_cfg.yaml`'s `cfg_file_path`)

### RL Policy

`control_module/policy/rl_walk.onnx` — 33-dim obs  = ang_vel(3)+gravity(3)+cmd(3)+joint_pos(8)+joint_vel(8)+action(8) → 8-dim action (delta joint positions). Inference runs every 4th control cycle (effective 250 Hz at 1 kHz loop).

## Project Structure

```
src/
├── protocols/my_ros2_proto/     # ROS 2 msg/srv definitions (JointCommand, JoyStickData)
├── module/
│   ├── joint_driver_module/     # CAN bus → Robstride actuators, transmission transforms
│   ├── joy_stick_module/        # SDL2 joystick + QP velocity limiter (qpOASES)
│   ├── control_module/          # RL + PD + state machine, ONNX inference, IMU est.
│   ├── oled_module/             # Oled logging and debugging module for efficient visual debugging
│   ├── imu_module/              # DM IMU L1 driver (serial, 921600 baud)
│   └── sim_module/              # MuJoCo physics + GLFW renderer
├── pkg/pkg1/                    # Package entry point, registers all modules
└── install/linux/bin/           # YAML configs + run scripts
    ├── run.sh                   # Real hardware mode launcher
    └── run_sim.sh               # Simulation mode launcher
```

## Important Notes

- **C++20 required**, GCC 13 enforced via `build.sh` (CC=/usr/bin/gcc-13)
- Module parameters (gains, limits, state machine config) are in YAML, not hardcoded
- The bundled third-party libs (ONNX Runtime, MuJoCo, qpOASES, DM IMU, Robstride) are in `control_module/third_party/`, `sim_module/third_party/`, `joy_stick_module/third_party/`, `joint_driver_module/robstride_controller/`, `imu_module/dm_imu_l1/`
