#ifndef OCCUPANCY_GRID_H
#define OCCUPANCY_GRID_H

#include <cinttypes>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <memory>
#include <vector>
#include "PlanningPoints.hpp"

namespace suas_planning{

    class GlobalWaypointPlanner {
    public:
        GlobalWaypointPlanner(ros::NodeHandle& nh, double resolution=1.0);
        GlobalWaypointPlanner(
            ros::NodeHandle& nh,
            double start_x,
            double start_y,
            double goal_x,
            double goal_y,
            std::vector<Obstacle> obstacles,
            double resolution=1.0
        );
        void SetStart(double x, double y);
        void SetGoal(double x, double y);
        void SetVehicleRadius(double radius);
        void AddObstacles(std::vector<Obstacle> obstacles);

    private:
        GlobalWaypointPlanner();
        double ComputeHeuristicCost(int x, int y);
        double ComputeMapSize();
        uint32_t InitMap();
        ros::NodeHandle& nh_;
        std::vector<int8_t> current_map_;
        ros::Subscriber map_subscriber_;
        uint32_t vehicle_radius_;
        uint32_t start_x_;
        uint32_t start_y_;
        uint32_t goal_x_;
        uint32_t goal_y_;

    };
}

#endif
