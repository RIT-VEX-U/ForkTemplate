#pragma once

// Device package
#include "core/device/cobs_device.h"
#include "core/device/wrapper_device.hpp"
#include "core/device/vdb/registry-controller.hpp"
#include "core/device/vdb/types.hpp"
#include "core/device/vdb/visitor.hpp"
#include "core/device/vdb/builtins.hpp"
#include "core/device/vdb/protocol.hpp"

// Subsystems package
#include "core/subsystems/fun/pl_mpeg.h"
#include "core/subsystems/fun/video.h"

#include "core/subsystems/odometry/odometry_3wheel.h"
#include "core/subsystems/odometry/odometry_base.h"
#include "core/subsystems/odometry/odometry_nwheel.h"
#include "core/subsystems/odometry/odometry_tank.h"

#include "core/subsystems/custom_encoder.h"
#include "core/subsystems/flywheel.h"
#include "core/subsystems/layout.h"
#include "core/subsystems/lift.h"
#include "core/subsystems/mecanum_drive.h"
#include "core/subsystems/screen.h"
#include "core/subsystems/tank_drive.h"

// Utils package
#include "core/utils/command_structure/auto_command.h"
#include "core/utils/command_structure/basic_command.h"
#include "core/utils/command_structure/command_controller.h"
#include "core/utils/command_structure/delay_command.h"
#include "core/utils/command_structure/drive_commands.h"
#include "core/utils/command_structure/flywheel_commands.h"

#include "core/utils/controls/state_space/linear_plant_inversion_feedforward.h"
#include "core/utils/controls/state_space/linear_quadratic_regulator.h"
#include "core/utils/controls/bang_bang.h"
#include "core/utils/controls/feedback_base.h"
#include "core/utils/controls/feedforward.h"
#include "core/utils/controls/motion_controller.h"
#include "core/utils/controls/pid.h"
#include "core/utils/controls/pidff.h"
#include "core/utils/controls/take_back_half.h"
#include "core/utils/controls/trapezoid_profile.h"

#include "core/utils/math/estimator/kalman_filter.h"
#include "core/utils/math/estimator/unscented_kalman_filter.h"
#include "core/utils/math/geometry/pose2d.h"
#include "core/utils/math/geometry/rotation2d.h"
#include "core/utils/math/geometry/transform2d.h"
#include "core/utils/math/geometry/translation2d.h"
#include "core/utils/math/geometry/twist2d.h"

#include "core/utils/math/numerical/numerical_integration.h"

#include "core/utils/math/systems/dare_solver.h"
#include "core/utils/math/systems/discretization.h"
#include "core/utils/math/systems/linear_system.h"

#include "core/utils/auto_chooser.h"
#include "core/utils/formatting.h"
#include "core/utils/generic_auto.h"
#include "core/utils/geometry.h"
#include "core/utils/graph_drawer.h"
#include "core/utils/interpolating_map.h"
#include "core/utils/logger.h"
#include "core/utils/math_util.h"
#include "core/utils/moving_average.h"
#include "core/utils/pure_pursuit.h"
#include "core/utils/serializer.h"
#include "core/utils/state_machine.h"

// Base package
#include "core/robot_specs.h"
