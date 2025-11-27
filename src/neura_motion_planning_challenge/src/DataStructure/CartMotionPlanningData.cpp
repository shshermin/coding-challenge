#include <neura_motion_planning_challenge/DataStructure/CartMotionPlanningData.h>

using namespace neura_motion_planning_challenge;

/**
 * @brief Default constructor for CartMotionPlanningData.
 * 
 * Initializes the pose to identity, time from start to 0.0, and header to default values.
 */
CartMotionPlanningData::CartMotionPlanningData()
    : pose(), time_from_start(0.0), header()
{
}

/**
 * @brief Constructor with pose, time, and header.
 * 
 * @param in_pose The Cartesian pose of the robot.
 * @param time The time from start of motion in seconds.
 * @param header The ROS message header containing frame and timestamp information.
 */
CartMotionPlanningData::CartMotionPlanningData(const geometry_msgs::Pose &in_pose,
                                               const double &time,
                                               const std_msgs::Header &header)
    : pose(in_pose), time_from_start(time), header(header)
{
}

/**
 * @brief Gets a copy of the current CartMotionPlanningData.
 * 
 * @return A deep copy of this CartMotionPlanningData object.
 */
CartMotionPlanningData CartMotionPlanningData::getData(void) const
{
    return CartMotionPlanningData(pose, time_from_start.toSec(), header);
}
