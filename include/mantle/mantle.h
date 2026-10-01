#pragma once

// Hardware Abstraction Layer (HAL) & Device-Independent Subsystems

#include "mantle/rtos.h"
#include "mantle/motor.h"
#include "mantle/sensor.h"
#include "mantle/controller.h"
#include "mantle/display/display.h"
#include "mantle/display/graph_drawer.h"
#include "mantle/display/legacy.h"
#include "mantle/display/legacy_bridge.h"
#include "mantle/display/screen_controller.h"

#include "mantle/subsystems/tank_drive.h"
#include "mantle/subsystems/flywheel.h"
#include "mantle/subsystems/lift.h"
#include "mantle/subsystems/state_machine.h"
#include "mantle/subsystems/custom_encoder.h"

#include "mantle/subsystems/odometry/odometry_base.h"
#include "mantle/subsystems/odometry/odometry_tank.h"
#include "mantle/subsystems/odometry/odometry_3wheel.h"
#include "mantle/subsystems/odometry/odometry_nwheel.h"
#include "mantle/subsystems/odometry/odometry_serial.h"

#include "mantle/comm/serial.h"
#include "mantle/comm/vdb/protocol.hpp"

#include "mantle/comm/vdb/types.hpp"
#include "mantle/comm/vdb/crc32.hpp"
#include "mantle/comm/vdb/builtins.hpp"

#include "mantle/commands/auto_command.h"
#include "mantle/commands/command_controller.h"
#include "mantle/commands/delay_command.h"
#include "mantle/commands/drive_commands.h"
#include "mantle/initializer.h"
#include "mantle/tuning.h"
