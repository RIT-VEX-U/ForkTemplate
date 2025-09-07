#include "core/utils/controls/boomerang.h"

PurePursuit::Path Boomerang::CreatePath(Pose2d start, Pose2d end, double lead, int numpoints, double radius){
    double hypotenuse = start.translation().distance(end.translation());
    Translation2d carrot_point(end.x() - (hypotenuse * sinf(end.rotation().radians()) * lead), end.y() - (hypotenuse * cosf(end.rotation().radians()) * lead));
    std::vector<Translation2d> pathpoints;
    printf("\n");
    printf("%.2f\t%.2f\n", carrot_point.x(), carrot_point.y());
    for(double i = 0; i < 1; i += 1/(double)numpoints){
        printf("i: %f\n", i);
        double one_minus_t = 1 - i;
        double x = one_minus_t * ((one_minus_t * start.x()) + (i * carrot_point.x()))
                    + (i * (one_minus_t * carrot_point.x() + (i * end.x())));
        double y = one_minus_t * ((one_minus_t * start.y()) + (i * carrot_point.y()))
                    + (i * (one_minus_t * carrot_point.y() + (i * end.y())));
        Translation2d point(x, y);
        pathpoints.push_back(point);
    }
    return PurePursuit::Path(pathpoints, radius);

};
