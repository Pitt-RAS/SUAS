#include "PlanningPoints.hpp"

namespace suas_planning {
    
Waypoint::Waypoint(double x, double y) :
    x_(round(x)),
    y_(round(y)) {}

}