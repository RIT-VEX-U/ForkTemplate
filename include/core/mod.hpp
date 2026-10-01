#pragma once

#include "core/core.h"

namespace core {

struct ModuleInfo {
    const char* name = "core";
    const char* description = "Platform-Agnostic Inner Layer (Math, Geometry, Kinematics, Controls, Estimators)";
    const char* version = "1.0.0";
};

inline ModuleInfo get_module_info() {
    return ModuleInfo{};
}

} // namespace core
