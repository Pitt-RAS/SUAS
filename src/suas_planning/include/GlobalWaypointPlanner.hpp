#ifndef OCCUPANCY_GRID_H
#define OCCUPANCY_GRID_H

#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <algorithm>
#include <cinttypes>
#include <memory>
#include <queue>
#include <unordered_set>
#include <vector>

#include "PlanningPoints.hpp"

namespace suas_planning {

class GlobalWaypointPlanner {
  public:
    GlobalWaypointPlanner(ros::NodeHandle &nh,
                          double start_x,
                          double start_y,
                          double goal_x,
                          double goal_y,
                          std::vector<Obstacle> obstacles,
                          std::vector<Waypoint> waypoints,
                          double radius = 1.0,
                          double resolution = 1.0);
    void SetStart(double x, double y);
    void SetGoal(double x, double y);
    void SetVehicleRadius(double radius);
    void AddObstacles(std::vector<Obstacle> obstacles);
    void AddWaypoints(std::vector<Waypoint> waypoints);
    std::vector<std::string> GeneratePlan();

    class Node {
      public:
        Node(
            int x, int y, const Waypoint &goal, int action, std::shared_ptr<Node> from, GlobalWaypointPlanner &planner);
        int x_;
        int y_;
        int action_;
        const Waypoint &goal_;
        std::shared_ptr<Node> from_;
        double h_cost_;
        double g_cost_;
        double f_cost_;
        int hash_code();
        bool IsGoal();

      private:
        GlobalWaypointPlanner &parent_;
        double ComputeHeuristicCost();
    };

  private:
    GlobalWaypointPlanner() = delete;
    void ExpandObstaclesByRadius(std::vector<Obstacle> &obstacles);
    unsigned int ComputeMapSize(std::vector<Obstacle> &obstacles, std::vector<Waypoint> &waypoints);
    unsigned int ComputeMapWidth(std::vector<Obstacle> &obstacles, std::vector<Waypoint> &waypoints);
    unsigned int ComputeMapHeight(std::vector<Obstacle> &obstacles, std::vector<Waypoint> &waypoints);
    int UpdateMap(std::vector<Obstacle> &obstacles);
    std::shared_ptr<GlobalWaypointPlanner::Node> AStarSearch(Waypoint start, Waypoint goal);
    ros::NodeHandle &nh_;
    std::vector<int8_t> current_map_;
    std::vector<Waypoint> waypoints_;
    ros::Subscriber map_subscriber_;
    ros::Publisher plan_publisher_;
    MapMetaInfo map_meta_;
    Waypoint start_;
    Waypoint goal_;
    double vehicle_radius_;
    double resolution_;
};

bool operator<(const GlobalWaypointPlanner::Node &lhs, const GlobalWaypointPlanner::Node &rhs);
bool operator>(const GlobalWaypointPlanner::Node &lhs, const GlobalWaypointPlanner::Node &rhs);
bool operator<=(const GlobalWaypointPlanner::Node &lhs, const GlobalWaypointPlanner::Node &rhs);
bool operator>=(const GlobalWaypointPlanner::Node &lhs, const GlobalWaypointPlanner::Node &rhs);

}  // namespace suas_planning

#endif
