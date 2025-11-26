#ifndef NEURA_MOTION_PLANNING_CHALLENGE_CART_MOTION_PLANNER_H
#define NEURA_MOTION_PLANNING_CHALLENGE_CART_MOTION_PLANNER_H

#include <neura_motion_planning_challenge/DataStructure/CartMotionPlanningData.h>
#include <neura_motion_planning_challenge/Sampling/SamplingStrategy.h>
#include <neura_motion_planning_challenge/InverseKinematics/IKStrategy.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <memory>

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
   * This method orchestrates the Cartesian path planning process using pluggable
   * sampling and IK strategies:
   *
   * Algorithm:
   * 1. Generate Cartesian waypoints using the provided SamplingStrategy
   * 2. Solve IK for each waypoint using the provided IKStrategy
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
   * @param sampler The sampling strategy to use for generating waypoints.
   *               Default: nullptr, which uses UniformSampler.
   *               Can pass any SamplingStrategy implementation (UniformSampler, AdaptiveSampler, etc).
   * @param ik_solver The IK strategy to use for solving joint configurations.
   *                 Default: nullptr, which uses JacobianIKSolver.
   *                 Can pass any IKStrategy implementation (JacobianIKSolver, AnalyticIKSolver, etc).
   *
   * @return A moveit_msgs::RobotTrajectory containing the joint trajectory that follows
   *         the Cartesian path. Returns an empty trajectory on failure.
   *
   * @throws std::exception If setup fails (robot model loading, sampling, IK setup, etc).
   *
   * @note If IK fails to converge for a waypoint but produces a reasonable approximation,
   *       planning continues. Only critical failures (joint limit violations, collisions)
   *       cause the entire plan to be rejected.
   *
   * @example
   *   // Using default UniformSampler and JacobianIKSolver
   *   auto traj = CartMotionPlanner::planNumericalCartesianPath(poseA, poseB, 10, true);
   *   
   *   // Using custom sampler and IK solver
   *   auto custom_sampler = std::make_shared<AdaptiveSampler>();
   *   auto custom_ik = std::make_shared<AnalyticIKSolver>();
   *   auto traj = CartMotionPlanner::planNumericalCartesianPath(
   *       poseA, poseB, 10, true, custom_sampler, custom_ik);
   */
  static moveit_msgs::RobotTrajectory planNumericalCartesianPath(
      const CartMotionPlanningData &waypoint_A,
      const CartMotionPlanningData &waypoint_B,
      int num_waypoints = 10,
      bool checkCollisions = false,
      const std::shared_ptr<SamplingStrategy>& sampler = nullptr,
      const std::shared_ptr<IKStrategy>& ik_solver = nullptr);
};

} // namespace neura_motion_planning_challenge

#endif // NEURA_MOTION_PLANNING_CHALLENGE_CART_MOTION_PLANNER_H