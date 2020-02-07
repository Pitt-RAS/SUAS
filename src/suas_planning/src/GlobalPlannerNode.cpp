//#include "GlobalPlannerNode.hpp"
//#include "GlobalWaypointPlanner.hpp"
//#include "PlanningPoints.hpp"

#include <ros/ros.h>
#include <ros/console.h>
#include <signal.h>

#include "PlanningPoints.hpp"
#include "GlobalWaypointPlanner.hpp"
    
int main(int argc, char** argv)
{
    //GlobalWaypointPlanner gwp;
    //MapMetaInfo mmi;
    //Obstacle ob;
    //CircularObstacle cob;
    //Waypoint wp;

    // TODO
    // ros node
    
    ros::init(argc, argv, "global_planner_node");
    ros::NodeHandle nh;

    ROS_DEBUG_ONCE("hello suas");

    double start_x;
    double start_y;
    double goal_x;
    double goal_y;

    start_x = start_y = 10;
    goal_x = goal_y = 100;

    std::vector<suas_planning::Obstacle> obstacles;
    std::vector<suas_planning::Waypoint> waypoints;

    obstacles.push_back(suas_planning::CircularObstacle(50, 50, 2));
    waypoints.push_back(suas_planning::Waypoint(35, 35));
    suas_planning::GlobalWaypointPlanner planner(
        nh,
        start_x,
        start_y,
        goal_x,
        goal_y,
        obstacles,
        waypoints);


    ros::spin();

    return 0;
}

