#pragma once

#include "aimrt_module_cpp_interface/logger/logger.h"

namespace mybipedal_deploy::rl_control_module {

void SetLogger(aimrt::logger::LoggerRef);
aimrt::logger::LoggerRef GetLogger();

}  // namespace mybipedal_deploy::rl_control_module
