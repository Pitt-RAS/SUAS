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
        GlobalWaypointPlanner(
            ros::NodeHandle& nh,
            double start_x,
            double start_y,
            double goal_x,
            double goal_y,
            std::vector<Obstacle> obstacles,
            double radius=1.0,
            double resolution=1.0
        );
        void SetStart(double x, double y);
        void SetGoal(double x, double y);
        void SetVehicleRadius(double radius);
        void AddObstacles(std::vector<Obstacle> obstacles);

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
        ros::Subscriber map_subscriber_;
        ros::Publisher plan_publisher_;
        MapMetaInfo::MapMetaInfo map_meta_;
        double resolution_;
        double vehicle_radius_;
        unsigned int start_x_;
        unsigned int start_y_;
        unsigned int goal_x_;
        unsigned int goal_y_;

};

}

#endif
