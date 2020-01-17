#include "GlobalPlannerNode.hpp"
#include "GlobalWaypointPlanner.hpp"
#include "PlanningPoints.hpp"

#include <ros/ros.h>
#include <signal.h>

namespace suas_planning
{
    
int main(int argc, char** argv)
{
    //GlobalWaypointPlanner gwp;
    MapMetaInfo mmi;
    Obstacle ob;
    CircularObstacle cob;
    Waypoint wp;

    // TODO
    // ros node
    
    ros::init(argc, argv, "global_planner_node");
    ros::NodeHandle nh;

    ros::spin();

    return 0;
}

} // namespace suas_planning
