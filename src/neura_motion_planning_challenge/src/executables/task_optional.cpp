// Optional Task: Cartesian path planning using numerical approach
// Plans a straight-line path from pose A to pose B without using MoveIt planners

#include <ros/ros.h>
#include <neura_motion_planning_challenge/motion_planning.h>
#include <neura_motion_planning_challenge/TrajectoryValidatorAndOptimizer.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/planning_scene/planning_scene.h>
#include <cmath>

using namespace neura_motion_planning_challenge;



int main(int argc, char** argv)
{
    ros::init(argc, argv, "task_optional_cartesian_planner");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
   
        
}
