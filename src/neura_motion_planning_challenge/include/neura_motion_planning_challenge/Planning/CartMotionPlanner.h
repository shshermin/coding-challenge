#ifndef NEURA_MOTION_PLANNING_CHALLENGE_CART_MOTION_PLANNER_H
#define NEURA_MOTION_PLANNING_CHALLENGE_CART_MOTION_PLANNER_H

#include <neura_motion_planning_challenge/DataStructure/CartMotionPlanningData.h>
#include <moveit_msgs/RobotTrajectory.h>

namespace neura_motion_planning_challenge
{

/**
 * @class CartMotionPlanner
 * @brief Plans a numerical Cartesian path from point A to point B using linear interpolation and IK solving.
 *
 * This class orchestrates the complete Cartesian path planning process:
 * 1. Linear interpolation between the start and end waypoints
 * 2. Segments the path into N discrete Cartesian waypoints
 * 3. Solves Inverse Kinematics (IK) for each waypoint
 * 4. Validates waypoints and path segments for collisions and joint limits
 * 5. Returns a joint trajectory following the Cartesian path
 *
 * The planner uses a Strategy Pattern for sampling and IK solving, allowing
 * different algorithms to be plugged in as needed.
 */
class CartMotionPlanner
{
public:
  /**
   * @brief Plans a numerical Cartesian path from waypoint A to waypoint B.
   *
   * This method uses uniform Cartesian sampling and Jacobian-based IK to plan
   * a path that smoothly moves from the start to the end waypoint.
   *
   * Algorithm:
   * 1. Generate uniformly-spaced Cartesian waypoints using UniformSampler
   * 2. Solve IK for each waypoint using JacobianIKSolver
   * 3. Validate each waypoint against joint limits and collisions
   * 4. Check collision-free paths between consecutive waypoints
   * 5. Build and validate the complete joint trajectory
   *
   * @param waypoint_A The starting Cartesian waypoint (point A).
   * @param waypoint_B The ending Cartesian waypoint (point B).
   * @param num_waypoints Number of waypoints to generate between A and B (including A and B).
   *                      Default: 10. Must be at least 2.
   * @param checkCollisions Whether to check for collisions along the path. Default: false.
   *                        When true, validates all waypoints and path segments.
   *
   * @return A moveit_msgs::RobotTrajectory containing the joint trajectory that follows
   *         the Cartesian path. Returns an empty trajectory on failure.
   *
   * @throws std::exception If setup fails (robot model loading, etc).
   *
   * @note If IK fails to converge for a waypoint but produces a reasonable approximation,
   *       planning continues. Only critical failures (joint limit violations, collisions)
   *       cause the entire plan to be rejected.
   */
  static moveit_msgs::RobotTrajectory planNumericalCartesianPath(
      const CartMotionPlanningData &waypoint_A,
      const CartMotionPlanningData &waypoint_B,
      int num_waypoints = 10,
      bool checkCollisions = false);
};

} // namespace neura_motion_planning_challenge

#endif // NEURA_MOTION_PLANNING_CHALLENGE_CART_MOTION_PLANNER_H