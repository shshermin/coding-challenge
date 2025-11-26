#ifndef NEURA_MOTION_PLANNING_CHALLENGE_TRAJECTORY_VISUALIZER_H
#define NEURA_MOTION_PLANNING_CHALLENGE_TRAJECTORY_VISUALIZER_H

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace neura_motion_planning_challenge
{

/**
 * @class TrajectoryVisualizer
 * @brief Static utility class for trajectory visualization and debugging.
 *
 * This class provides methods for visualizing trajectories in RViz and
 * logging trajectory details for debugging purposes.
 *
 * @note All methods are static. No instantiation required - use TrajectoryVisualizer::methodName().
 */
class TrajectoryVisualizer
{
public:
  // Delete constructors to enforce static-only usage
  TrajectoryVisualizer() = delete;
  TrajectoryVisualizer(const TrajectoryVisualizer&) = delete;
  TrajectoryVisualizer& operator=(const TrajectoryVisualizer&) = delete;

  /**
   * @brief Publishes a planned trajectory to RViz for visualization.
   * @param plan The MoveIt plan containing the trajectory to visualize.
   * @param display_publisher ROS publisher for displaying trajectories.
   */
  static void publishTrajectoryToRViz(const moveit::planning_interface::MoveGroupInterface::Plan& plan,
                                       ros::Publisher& display_publisher);
  
  /**
   * @brief Logs trajectory details for debugging purposes.
   * @param trajectory The joint trajectory to log.
   */
  static void logTrajectoryDetails(const trajectory_msgs::JointTrajectory& trajectory);
};

} // namespace neura_motion_planning_challenge

#endif // NEURA_MOTION_PLANNING_CHALLENGE_TRAJECTORY_VISUALIZER_H
