#include "PlanningPoints.hpp"

namespace suas_planning {

Waypoint::Waypoint(double x, double y) : x_(round(x)), y_(round(y)) {}

bool operator==(const Waypoint& lhs, const Waypoint& rhs) { return (lhs.x_ == rhs.x_) && (lhs.y_ == rhs.y_); }
bool operator!=(const Waypoint& lhs, const Waypoint& rhs) { return !(lhs == rhs); }

}  // namespace suas_planning
