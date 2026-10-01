#pragma once

// Platform-Agnostic Core Inner Engine (ZERO VEX / PROS dependencies)

// Geometry
#include "core/geometry/eigen_interface.h"
#include "core/geometry/point2d.h"
#include "core/geometry/translation2d.h"
#include "core/geometry/rotation2d.h"
#include "core/geometry/transform2d.h"
#include "core/geometry/twist2d.h"
#include "core/geometry/pose2d.h"
#include "core/geometry/rect.h"

// Controls
#include "core/controls/feedback_base.h"
#include "core/controls/pid.h"
#include "core/controls/pidff.h"
#include "core/controls/bang_bang.h"
#include "core/controls/feedforward.h"
#include "core/controls/trapezoid_profile.h"
#include "core/controls/motion_controller.h"
#include "core/controls/state_space/linear_system.h"
#include "core/controls/state_space/linear_quadratic_regulator.h"
#include "core/controls/state_space/linear_plant_inversion_feedforward.h"
#include "core/controls/state_space/dare_solver.h"
#include "core/controls/state_space/discretization.h"

// Math & Estimation
#include "core/math/math_util.h"
#include "core/math/numerical_integration.h"
#include "core/math/kalman_filter.h"
#include "core/math/unscented_kalman_filter.h"

// Pathing
#include "core/pathing/pure_pursuit.h"

// Filter
#include "core/filter/moving_average.h"

// Utilities
#include "core/utils/formatting.h"
#include "core/utils/interpolating_map.h"
#include "core/utils/time.h"
#include "core/utils/units.h"

