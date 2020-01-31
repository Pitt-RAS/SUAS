//#include "GlobalPlannerNode.hpp"
//#include "GlobalWaypointPlanner.hpp"
//#include "PlanningPoints.hpp"

#include <ros/ros.h>
#include <ros/console.h>
#include <signal.h>

//namespace suas_planning
//{
    
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

    std::cout << "hello suas" << std::endl;

    ros::spin();

    return 0;
}

//} // namespace suas_planning
