#ifndef OCCUPANCY_GRID_H
#define OCCUPANCY_GRID_H

#include <cinttypes>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <memory>
#include <vector>
#include <tuple>
#include "PlanningPoints.hpp"

namespace suas_planning{

bool operator<(const GlobalWaypointPlanner::Node& lhs, const GlobalWaypointPlanner::Node& rhs);
bool operator>(const GlobalWaypointPlanner::Node& lhs, const GlobalWaypointPlanner::Node& rhs);
bool operator<=(const GlobalWaypointPlanner::Node& lhs, const GlobalWaypointPlanner::Node& rhs);
bool operator>=(const GlobalWaypointPlanner::Node& lhs, const GlobalWaypointPlanner::Node& rhs);

class GlobalWaypointPlanner {
    public:
        GlobalWaypointPlanner(
            ros::NodeHandle& nh,
            double start_x,
            double start_y,
            double goal_x,
            double goal_y,
            std::vector<Obstacle> obstacles,
            std::vector<Waypoint> waypoints,
            double radius=1.0,
            double resolution=1.0
        );
        void SetStart(double x, double y);
        void SetGoal(double x, double y);
        void SetVehicleRadius(double radius);
        void AddObstacles(std::vector<Obstacle> obstacles);
        void AddWaypoints(std::vector<Waypoint> waypoints);
        std::vector<std::string> GeneratePlan();

        class Node {
            public:
                Node(int x, int y, Waypoint& goal, int action, Node& from, GlobalWaypointPlanner& planner);
                int x_;
                int y_;
                int action_;
                Waypoint& goal_;
                Node& from_;
                double h_cost_;
                double g_cost_;
                double f_cost_;
                int hash();
            private:
                GlobalWaypointPlanner& parent_;
                double ComputeHeuristicCost();
        };

    private:
        GlobalWaypointPlanner();
        double ComputeHeuristicCost(int x, int y);
        void ExpandObstaclesByRadius(std::vector<Obstacle>& obstacles);
        unsigned int ComputeMapSize(std::vector<Obstacle>& obstacles);
        unsigned int ComputeMapWidth(std::vector<Obstacle>& obstacles);
        unsigned int ComputeMapHeight(std::vector<Obstacle>& obstacles);
        int UpdateMap(std::vector<Obstacle>& obstacles);
        ros::NodeHandle& nh_;
        std::vector<int8_t> current_map_;
        std::vector<Waypoint> waypoints_;
        ros::Subscriber map_subscriber_;
        ros::Publisher plan_publisher_;
        MapMetaInfo::MapMetaInfo map_meta_;
        Waypoint::Waypoint start_;
        Waypoint::Waypoint goal_;
        double resolution_;
        double vehicle_radius_;

};

}

#endif
