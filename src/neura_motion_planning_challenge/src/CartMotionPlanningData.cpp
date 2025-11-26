#include <neura_motion_planning_challenge/CartMotionPlanningData.h>

using namespace neura_motion_planning_challenge;

CartMotionPlanningData::CartMotionPlanningData()
    : pose(), time_from_start(0.0), header()
{
}

CartMotionPlanningData::CartMotionPlanningData(const geometry_msgs::Pose &in_pose,
                                               const double &time,
                                               const std_msgs::Header &header)
    : pose(in_pose), time_from_start(time), header(header)
{
}

CartMotionPlanningData CartMotionPlanningData::getData(void) const
{
    return CartMotionPlanningData(pose, time_from_start.toSec(), header);
}
