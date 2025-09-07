#pragma once
#include "core/utils/math/geometry/pose2d.h"
#include "core/utils/pure_pursuit.h"
#include <vector>
#include <cmath>

class Boomerang{
    public:
    static PurePursuit::Path CreatePath(Pose2d start, Pose2d end, double lead, int numpoints, double radius);


};