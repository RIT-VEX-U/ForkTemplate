#pragma once

#include "crust/crust.h"
#include "crust/comm/cobs_device.h"
#include "crust/comm/wrapper_device.hpp"

namespace crust {

/// Initializes hardware clock and delay hooks into core::Timer
void init();

const char* crust_version_string();

} // namespace crust
