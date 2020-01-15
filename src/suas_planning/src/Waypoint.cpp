#include "PlanningPoints.hpp"

namespace suas_planning {
    Waypoint::Waypoint(double x, double y) :
        x_((unsigned int) round(x)),
        y_((unsigned int) round(y)) {}
}