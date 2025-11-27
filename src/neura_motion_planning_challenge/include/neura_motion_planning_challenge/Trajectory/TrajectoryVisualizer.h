#ifndef NEURA_MOTION_PLANNING_CHALLENGE_TRAJECTORY_VISUALIZER_H
#define NEURA_MOTION_PLANNING_CHALLENGE_TRAJECTORY_VISUALIZER_H

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>

namespace neura_motion_planning_challenge
{
// Forward declaration
class CartMotionPlanningData;

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

  /**
   * @brief Visualizes and logs a planned trajectory in RViz.
   * Combines publishing to RViz, logging details, and logging success message.
   * @param plan The MoveIt plan containing the trajectory to visualize.
   * @param display_publisher ROS publisher for displaying trajectories.
   * @param method_name Name of the planning method (for logging).
   */
  static void logAndVisualizeTrajectory(const moveit::planning_interface::MoveGroupInterface::Plan& plan,
                                         ros::Publisher& display_publisher,
                                         const std::string& method_name);

  /**
   * @brief Visualizes a joint trajectory in RViz by publishing it for animation.
   * @param joint_trajectory The joint trajectory to visualize.
   * @param display_publisher ROS publisher for displaying trajectories.
   * @param move_group_ptr Shared pointer to the MoveGroup interface for getting current state.
   * @return true if visualization succeeded, false otherwise.
   */
  static bool visualizeJointTrajectory(const moveit_msgs::RobotTrajectoryPtr& joint_trajectory,
                                        ros::Publisher& display_publisher,
                                        const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group_ptr);

  /**
   * @brief Visualizes a sequence of Cartesian waypoints in RViz as colored sphere markers.
   * Creates colored sphere markers along the Cartesian path:
   * - Green sphere marks the start waypoint
   * - Red sphere marks the end waypoint
   * - Yellow spheres mark intermediate waypoints
   * @param cart_pose_array Vector of Cartesian waypoints to visualize.
   * @return true if visualization succeeded, false otherwise.
   */
  static bool visualizeCartTrajectory(const std::vector<CartMotionPlanningData>& cart_pose_array);
};

} // namespace neura_motion_planning_challenge

#endif // NEURA_MOTION_PLANNING_CHALLENGE_TRAJECTORY_VISUALIZER_H
