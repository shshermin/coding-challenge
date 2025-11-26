#ifndef NEURA_MOTION_PLANNING_CHALLENGE_TRAJECTORY_VALIDATOR_AND_OPTIMIZER_H
#define NEURA_MOTION_PLANNING_CHALLENGE_TRAJECTORY_VALIDATOR_AND_OPTIMIZER_H

#include <trajectory_msgs/JointTrajectory.h>
#include <string>
#include <moveit/planning_scene/planning_scene.h>

namespace neura_motion_planning_challenge
{

/**
 * @class TrajectoryValidatorAndOptimizer
 * @brief Static utility class for trajectory validation, optimization, and analysis.
 *
 * This class provides methods for:
 * - Validating trajectories against joint limits and collision constraints
 * - Optimizing trajectories to respect velocity/acceleration limits
 * - Plotting and comparing trajectories for analysis
 *
 * @note All methods are static. No instantiation required.
 */
class TrajectoryValidatorAndOptimizer {
public:
  /**
   * @brief Validates a trajectory against joint limits, velocity limits, and collision constraints.
   * 
   * @param traj The trajectory to validate
   * @param group_name The planning group name (e.g., "arm")
   * @param planning_scene Optional planning scene for collision checking
   * @param error_out Optional pointer to store error message
   * @return true if trajectory is valid, false otherwise
   */
  static bool validateTrajectory(const trajectory_msgs::JointTrajectory& traj,
                                 const std::string& group_name,
                                 const planning_scene::PlanningScenePtr& planning_scene = planning_scene::PlanningScenePtr(),
                                 std::string* error_out = nullptr);
  
  /**
   * @brief Optimizes a trajectory to respect velocity/acceleration limits and smooth motion.
   * 
   * @param trajectory The trajectory to optimize (will be modified in place)
   * @param group_name The planning group name (e.g., "arm")
   * @return true if optimization succeeded, false otherwise
   */
  static bool optimizeTrajectory(trajectory_msgs::JointTrajectory& trajectory, 
                                 const std::string& group_name = "arm");
  
  /**
   * @brief Plots and compares two trajectories (original vs optimized) using Python matplotlib.
   * 
   * @param original_traj The original trajectory
   * @param optimized_traj The optimized trajectory
   * @param output_path Path where the plot PNG will be saved
   * @return true if plot was successfully generated, false otherwise
   */
  static bool plotTrajectoryComparison(const trajectory_msgs::JointTrajectory& original_traj,
                                       const trajectory_msgs::JointTrajectory& optimized_traj,
                                       const std::string& output_path);
};

} // namespace neura_motion_planning_challenge

#endif // NEURA_MOTION_PLANNING_CHALLENGE_TRAJECTORY_VALIDATOR_AND_OPTIMIZER_H
