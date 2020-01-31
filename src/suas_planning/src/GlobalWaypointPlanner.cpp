#include "GlobalWaypointPlanner.hpp"

namespace std {
    
    template<> struct hash<suas_planning::GlobalWaypointPlanner::Node> {
        std::size_t operator()(const suas_planning::GlobalWaypointPlanner::Node& node) const noexcept {
            std::size_t h1 = (node.x_ + 0x7261735f) >> 1;
            std:: size_t h2 = (node.y_ + 0x7261735f) >> 1;
            return (h1 ^ h2) + (h1 << 6) + (h1 >> 2);
        }
    };
    
} // namespace std for std::hash injection

namespace suas_planning {

static const int NUM_ACTIONS = 8;
static const int dX[NUM_ACTIONS] = {0, 0, 1, 1, 1, -1, -1, -1};
static const int dY[NUM_ACTIONS] = {1, -1, 0, 1, -1, 0, 1, -1};
static const double action_cost[NUM_ACTIONS] = {1, 1, 1, M_SQRT2, M_SQRT2, 1, M_SQRT2, M_SQRT2};

GlobalWaypointPlanner::GlobalWaypointPlanner(
        ros::NodeHandle& nh,
        double start_x,
        double start_y,
        double goal_x,
        double goal_y,
        std::vector<Obstacle> obstacles,
        std::vector<Waypoint> waypoints,
        double radius,
        double resolution) :
            nh_(nh),
            current_map_(ComputeMapSize(obstacles)),
            waypoints_(waypoints),
            map_meta_(current_map_, ComputeMapWidth(obstacles), ComputeMapHeight(obstacles)),
            start_((int) round(start_x), (int) round(start_y)),
            goal_((int) round(goal_x), (int) round(goal_y)),
            vehicle_radius_(radius),
            resolution_(resolution) {
        UpdateMap(obstacles);
}

std::vector<std::string> GeneratePlan() {
    return {""};
}

void GlobalWaypointPlanner::ExpandObstaclesByRadius(std::vector<Obstacle>& obstacles) {
    std::vector<Obstacle>::iterator obstacle_it;
    for (obstacle_it = obstacles.begin(); obstacle_it != obstacles.end(); obstacle_it++) {
        // Expand obstacle by vehicle radius so we can treat vehicle as point object
        Obstacle curr_obstacle = *obstacle_it;
        curr_obstacle.ExpandSize(vehicle_radius_);
    }
}

// not meant to be used prior to ComputeMapSize
// This is probably a terrible way of doing this, maybe just have obstacles be copied instead
// of being passed by reference.
unsigned int GlobalWaypointPlanner::ComputeMapWidth(std::vector<Obstacle>& obstacles) {
    int min_x = start_.x_;
    int max_x = goal_.x_;

    // iterate through obstacles to compute maximum size
    std::vector<Obstacle>::iterator obstacle_it;
    for (obstacle_it = obstacles.begin(); obstacle_it != obstacles.end(); obstacle_it++) {
        Obstacle curr_obstacle = *obstacle_it;
        int current_min_x = curr_obstacle.GetMinX();
        int current_max_x = curr_obstacle.GetMaxX();
        if (current_min_x < min_x) {
            min_x = current_min_x;
        }
        if (current_max_x > max_x) {
            max_x = current_max_x;
        }
    }

    return abs(max_x - min_x);
}

// not meant to be used prior to ComputeMapSize
// This is probably a terrible way of doing this, maybe just have obstacles be copied instead
// of being passed by reference.
unsigned int GlobalWaypointPlanner::ComputeMapHeight(std::vector<Obstacle>& obstacles) {
    int min_y = start_.y_;
    int max_y = goal_.y_;

    // iterate through obstacles to compute maximum size
    std::vector<Obstacle>::iterator obstacle_it;
    for (obstacle_it = obstacles.begin(); obstacle_it != obstacles.end(); obstacle_it++) {
        Obstacle curr_obstacle = *obstacle_it;
        int current_max_y = curr_obstacle.GetMaxY();
        int current_min_y = curr_obstacle.GetMinY();
        if (current_min_y < min_y) {
            min_y = current_min_y;
        }
        if (current_max_y > max_y) {
            max_y = current_max_y;
        }
    }

    return abs(max_y - min_y);
}

// note to self: this is slightly flawed probably, everything needs to be reference to an absolute
// reference, which I don't know yet; also this could be O(n) but for now we leave it as O(n^2)
unsigned int GlobalWaypointPlanner::ComputeMapSize(std::vector<Obstacle>& obstacles) {
    ExpandObstaclesByRadius(obstacles);
    int width = ComputeMapWidth(obstacles);
    int height = ComputeMapHeight(obstacles);
    return width * height;
}

int GlobalWaypointPlanner::UpdateMap(std::vector<Obstacle>& obstacles) {
    std::vector<Obstacle>::iterator obstacle_it;
    for (obstacle_it = obstacles.begin(); obstacle_it != obstacles.end(); obstacle_it++) {
        Obstacle curr_obstacle = *obstacle_it;
        curr_obstacle.PlotObstacle(current_map_, map_meta_);
    }
    return obstacles.size();
}

GlobalWaypointPlanner::Node::Node(int x, int y, Waypoint& goal, int action, Node& from, GlobalWaypointPlanner& planner) :
    x_(x),
    y_(y),
    action_(action),
    goal_(goal),
    from_(from),
    g_cost_(action_cost[action]),
    parent_(planner) {
        h_cost_ = ComputeHeuristicCost();
        f_cost_ = g_cost_ + h_cost_;
}

double GlobalWaypointPlanner::Node::ComputeHeuristicCost() { 
    double dx = (double) (goal_.x_ - x_);
    double dy = (double) (goal_.y_ - y_);
    return sqrt((dx * dx) + (dy * dy));
}

int GlobalWaypointPlanner::Node::hash_code() {
    int h1 = (x_ + 0x7261735f) >> 1;
    int h2 = ((y_ + 0x7261735f) >> 1);
    return (h1 ^ h2) + (h1 << 6) + (h1 >> 2);
}

// Need to do proper floating point comparison before full release.
// Will implement ULP most likely as we will be dealing with mostly
// Non-zero positive values.
bool operator<(const GlobalWaypointPlanner::Node& lhs, const GlobalWaypointPlanner::Node& rhs) {
        return lhs.f_cost_ < rhs.f_cost_;
}

bool operator>(const GlobalWaypointPlanner::Node& lhs, const GlobalWaypointPlanner::Node& rhs) {
    return rhs < lhs;
}

bool operator<=(const GlobalWaypointPlanner::Node& lhs, const GlobalWaypointPlanner::Node& rhs) {
    return !(lhs > rhs);
}

bool operator>=(const GlobalWaypointPlanner::Node& lhs, const GlobalWaypointPlanner::Node& rhs) {
    return !(lhs < rhs);
}

}
