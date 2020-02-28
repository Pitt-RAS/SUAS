#include "GlobalWaypointPlanner.hpp"

namespace std {

// Custom hash for Node; hex number if a magic number to make sure more uniform distribution
template <>
struct hash<suas_planning::GlobalWaypointPlanner::Node> {
    std::size_t operator()(const suas_planning::GlobalWaypointPlanner::Node &node) const noexcept {
        std::size_t h1 = (node.x_ + 0x7261735f) >> 1;
        std::size_t h2 = (node.y_ + 0x7261735f) >> 1;
        std::size_t h3 = (node.action_ + 0x7261735f) >> 1;
        std::size_t temp = (h1 ^ h2) + (h1 << 6) + (h1 >> 2);
        return (temp ^ h3) + (temp << 6) + (h3 >> 2);
    }
};

template <>
struct equal_to<suas_planning::GlobalWaypointPlanner::Node> {
    bool operator()(const suas_planning::GlobalWaypointPlanner::Node &lhs,
                    const suas_planning::GlobalWaypointPlanner::Node &rhs) const noexcept {
        bool x = lhs.x_ == rhs.y_;
        bool y = lhs.y_ == rhs.y_;
        bool action = lhs.action_ == rhs.action_;
        bool from = lhs.from_ == rhs.from_;
        bool goal = lhs.goal_ == rhs.goal_;
        return x && y && action && from && goal;
    }
};

}  // namespace std

namespace suas_planning {

static const int NO_ACTION = -1;
static const int NUM_ACTIONS = 8;
static const int dX[NUM_ACTIONS] = {0, 0, 1, 1, 1, -1, -1, -1};
static const int dY[NUM_ACTIONS] = {1, -1, 0, 1, -1, 0, 1, -1};
static const double action_cost[NUM_ACTIONS] = {1, 1, 1, M_SQRT2, M_SQRT2, 1, M_SQRT2, M_SQRT2};

GlobalWaypointPlanner::GlobalWaypointPlanner(ros::NodeHandle &nh,
                                             double start_x,
                                             double start_y,
                                             double goal_x,
                                             double goal_y,
                                             std::vector<Obstacle> obstacles,
                                             std::vector<Waypoint> waypoints,
                                             double radius,
                                             double resolution)
    : nh_(nh),
      current_map_(ComputeMapSize(obstacles, waypoints)),
      waypoints_(waypoints),
      map_meta_(current_map_, ComputeMapWidth(obstacles, waypoints), ComputeMapHeight(obstacles, waypoints)),
      start_((int)round(start_x), (int)round(start_y)),
      goal_((int)round(goal_x), (int)round(goal_y)),
      vehicle_radius_(radius),
      resolution_(resolution) {
    UpdateMap(obstacles);
}

/*
 * Generates a route/plan for waypoints and goals
 */
std::vector<std::string> GlobalWaypointPlanner::GeneratePlan() {
    Waypoint current_start = start_;
    std::queue<Waypoint, std::deque<Waypoint>> sub_goals;
    std::vector<std::string> sub_plans;
    if (!sub_goals.size()) {
        for (const Waypoint &waypoint : waypoints_) {
            sub_goals.push(waypoint);
        }
    }
    sub_goals.push(goal_);
    // Start iterating through the sub_goals and execute a from waypoints
    std::vector<std::string> action_strings;
    while (sub_goals.size() > 0) {
        // Get first item goal off and pop goal off stack
        Waypoint current_goal = sub_goals.front();
        sub_goals.pop();
        std::shared_ptr<GlobalWaypointPlanner::Node> goal = AStarSearch(current_start, current_goal);

        // start populating waypoints/generating waypoint plans
        std::vector<std::string> waypoint_data;
        Waypoint current_waypoint(goal->x_, goal->y_);
        int current_action = goal->action_;
        int path_length = 0;
        std::shared_ptr<Node> current_node = goal->from_;
        while (current_node != nullptr) {
            if (current_node->action_ == current_action) {
                path_length += 1;
            } else {
                std::stringstream plan_stream;
                plan_stream <<  "{x:" << current_waypoint.x_ << ",y:" << current_waypoint.y_
                            << ",action:" << current_action << ",length:" << path_length << "}";
                waypoint_data.push_back(plan_stream.str());
                current_waypoint.x_ = current_node->x_;
                current_waypoint.y_ = current_node->y_;
                current_action = current_node->action_;
                path_length = 0;
            }
            current_node = current_node->from_;
        }
        action_strings.insert(action_strings.end(), waypoint_data.begin(), waypoint_data.end());
    }

    return action_strings;
}

/*
 * Implements offline A* given start and sub goal
 * Note due to the use of a unordered set with a priority queue as the frontier (probably should just subclass this);
 * You need to first do operations on the pq_frontier unordered_set before pq (assuming you need to use std::move)
 */
std::shared_ptr<GlobalWaypointPlanner::Node> GlobalWaypointPlanner::AStarSearch(Waypoint start, Waypoint goal) {
    // lambda function to compare Node objects
    auto node_compare = [](const std::shared_ptr<Node> &a, const ::std::shared_ptr<Node> &b) { return *a < *b; };
    // Creates a Min Heap for A*/Shortest Path Search
    std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, decltype(node_compare)> pq(
        node_compare);
    std::unordered_set<Node, std::hash<Node>, std::equal_to<Node>>
        pq_frontier;  // just a hack so we check the frontier for nodes ()
    std::unordered_set<Node, std::hash<Node>, std::equal_to<Node>> explored;

    // create node using starting waypoint and push that onto related data sturcutres
    auto current = std::make_shared<Node>(start_.x_, start.y_, goal, NO_ACTION, nullptr, *this);
    pq_frontier.insert(*current);
    pq.push(std::move(current));
    while (pq.size()) {
        // Pop the current node off of PQ and Frontier
        std::shared_ptr<Node> current_node = pq.top();
        pq_frontier.erase(*current_node);
        pq.pop();

        // Insert into explored
        explored.insert(*current_node);
        for (int action = 0; action < NUM_ACTIONS; action++) {
            // Expand children for current node
            int new_x = current_node->x_ + dX[action];
            int new_y = current_node->y_ + dY[action];
            bool in_bounds = Obstacle::CheckGridBounds(current_map_, new_x, new_y, map_meta_);
            bool is_free = Obstacle::IsFree(current_map_, new_x, new_y, map_meta_);
            if (in_bounds && is_free) {
                auto child_node = std::make_shared<Node>(new_x, new_y, goal, action, current, *this);
                if (explored.find(*child_node) != explored.end() ||
                    pq_frontier.find(*child_node) != pq_frontier.end()) {
                    if (child_node->IsGoal()) {
                        // Found a solution, return it back up for processing
                        return child_node;
                    }
                    pq_frontier.insert(*child_node);
                    pq.push(std::move(child_node));
                }
            }
        }
    }

    return nullptr;
}

void GlobalWaypointPlanner::ExpandObstaclesByRadius(std::vector<Obstacle> &obstacles) {
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
unsigned int GlobalWaypointPlanner::ComputeMapWidth(std::vector<Obstacle> &obstacles,
                                                    std::vector<Waypoint> &waypoints) {
    std::vector<int> x_points = {start_.x_, goal_.x_};
    // push back all y values for obstacles
    for (Obstacle &obstacle : obstacles) {
        x_points.push_back(obstacle.GetMinX());
        x_points.push_back(obstacle.GetMaxX());
    }
    // push back all y values for waypoints
    for (Waypoint &waypoint : waypoints_) {
        x_points.push_back(waypoint.x_);
    }
    // find smallest and largest x to compute range
    const auto min_x = std::min_element(x_points.begin(), x_points.end());
    const auto max_x = std::max_element(x_points.begin(), x_points.end());
    return abs(*max_x - *min_x);
}

// not meant to be used prior to ComputeMapSize
// This is probably a terrible way of doing this, maybe just have obstacles be copied instead
// of being passed by reference.
unsigned int GlobalWaypointPlanner::ComputeMapHeight(std::vector<Obstacle> &obstacles,
                                                     std::vector<Waypoint> &waypoints) {
    std::vector<int> y_points = {start_.y_, goal_.y_};
    // push back all y values for obstacles
    for (Obstacle &obstacle : obstacles) {
        y_points.push_back(obstacle.GetMinY());
        y_points.push_back(obstacle.GetMaxY());
    }
    // push back all y values for waypoints
    for (Waypoint &waypoint : waypoints_) {
        y_points.push_back(waypoint.y_);
    }
    // find smallest and largest y to compute range
    const auto min_y = std::min_element(y_points.begin(), y_points.end());
    const auto max_y = std::max_element(y_points.begin(), y_points.end());
    return abs(*max_y - *min_y);
}

// note to self: this is slightly flawed probably, everything needs to be reference to an absolute
// reference, which I don't know yet; also this could be O(n) but for now we leave it as O(n^2)
unsigned int GlobalWaypointPlanner::ComputeMapSize(std::vector<Obstacle> &obstacles, std::vector<Waypoint> &waypoints) {
    ExpandObstaclesByRadius(obstacles);
    int width = ComputeMapWidth(obstacles, waypoints);
    int height = ComputeMapHeight(obstacles, waypoints);
    return width * height;
}

int GlobalWaypointPlanner::UpdateMap(std::vector<Obstacle> &obstacles) {
    std::vector<Obstacle>::iterator obstacle_it;
    for (obstacle_it = obstacles.begin(); obstacle_it != obstacles.end(); obstacle_it++) {
        Obstacle curr_obstacle = *obstacle_it;
        curr_obstacle.PlotObstacle(current_map_, map_meta_);
    }
    return obstacles.size();
}

GlobalWaypointPlanner::Node::Node(
    int x, int y, const Waypoint &goal, int action, std::shared_ptr<Node> from, GlobalWaypointPlanner &planner)
    : x_(x), y_(y), action_(action), goal_(goal), from_(from), parent_(planner) {
    if (action != NO_ACTION) {
        g_cost_ = action_cost[action];
        h_cost_ = ComputeHeuristicCost();
        f_cost_ = g_cost_ + h_cost_;
    } else {
        // Special case for start node
        f_cost_ = h_cost_ = g_cost_ = 0;
    }
}

double GlobalWaypointPlanner::Node::ComputeHeuristicCost() {
    double dx = static_cast<double>(goal_.x_ - x_);
    double dy = static_cast<double>(goal_.y_ - y_);
    return sqrt((dx * dx) + (dy * dy));
}

int GlobalWaypointPlanner::Node::hash_code() { return std::hash<Node>{}(*this); }

bool GlobalWaypointPlanner::Node::IsGoal() { return (x_ == goal_.x_) && (y_ == goal_.y_); }

// Need to do proper floating point comparison before full release.
// Will implement ULP most likely as we will be dealing with mostly
// Non-zero positive values.
bool operator<(const GlobalWaypointPlanner::Node &lhs, const GlobalWaypointPlanner::Node &rhs) {
    return lhs.f_cost_ < rhs.f_cost_;
}

bool operator>(const GlobalWaypointPlanner::Node &lhs, const GlobalWaypointPlanner::Node &rhs) { return rhs < lhs; }

bool operator<=(const GlobalWaypointPlanner::Node &lhs, const GlobalWaypointPlanner::Node &rhs) { return !(lhs > rhs); }

bool operator>=(const GlobalWaypointPlanner::Node &lhs, const GlobalWaypointPlanner::Node &rhs) { return !(lhs < rhs); }

}  // namespace suas_planning
