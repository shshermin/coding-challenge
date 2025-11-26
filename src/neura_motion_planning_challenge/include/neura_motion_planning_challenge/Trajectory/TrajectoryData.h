#ifndef NEURA_MOTION_PLANNING_CHALLENGE_TRAJECTORY_DATA_H
#define NEURA_MOTION_PLANNING_CHALLENGE_TRAJECTORY_DATA_H

#include <trajectory_msgs/JointTrajectory.h>
#include <neura_motion_planning_challenge/DataStructure/CartMotionPlanningData.h>

namespace neura_motion_planning_challenge
{

/**
 * @brief Stores both joint and Cartesian trajectory data.
 *
 * This struct combines a `trajectory_msgs::JointTrajectory` and a `CartMotionPlanningData`
 * within a single structure. It provides convenient access to both joint-space and
 * Cartesian-space representations of a planned robot motion.
 */
struct TrajectoryData
{
public:
  /**
   * @brief Get the joint trajectory.
   */
  trajectory_msgs::JointTrajectory& getJointTrajectory() { return joint_trajectory; }
  const trajectory_msgs::JointTrajectory& getJointTrajectory() const { return joint_trajectory; }
  
  /**
   * @brief Get the Cartesian trajectory.
   */
  CartMotionPlanningData& getCartTrajectory() { return cart_trajectory; }
  const CartMotionPlanningData& getCartTrajectory() const { return cart_trajectory; }
  
  /**
   * @brief Set the joint trajectory.
   */
  void setJointTrajectory(const trajectory_msgs::JointTrajectory& traj) { joint_trajectory = traj; }
  
  /**
   * @brief Set the Cartesian trajectory.
   */
  void setCartTrajectory(const CartMotionPlanningData& cart) { cart_trajectory = cart; }

private:
  /**
   * @brief The planned joint trajectory in joint space.
   */
  trajectory_msgs::JointTrajectory joint_trajectory;

  /**
   * @brief A single Cartesian waypoint representing the trajectory.
   *
   * This can be used for visualization or basic analysis.
   */
  CartMotionPlanningData cart_trajectory;
};

} // namespace neura_motion_planning_challenge

#endif // NEURA_MOTION_PLANNING_CHALLENGE_TRAJECTORY_DATA_H
