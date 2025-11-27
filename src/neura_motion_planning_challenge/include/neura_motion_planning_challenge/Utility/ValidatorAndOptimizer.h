#ifndef NEURA_MOTION_PLANNING_CHALLENGE_VALIDATOR_AND_OPTIMIZER_H
#define NEURA_MOTION_PLANNING_CHALLENGE_VALIDATOR_AND_OPTIMIZER_H

#include <trajectory_msgs/JointTrajectory.h>
#include <neura_motion_planning_challenge/DataStructure/PlanMetaData.h>
#include <string>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>

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
   * @param robot_model Optional robot model to avoid reloading
   * @return true if trajectory is valid, false otherwise
   */
  static bool validateTrajectory(const trajectory_msgs::JointTrajectory& traj,
                                 const std::string& group_name,
                                 const planning_scene::PlanningScenePtr& planning_scene = planning_scene::PlanningScenePtr(),
                                 std::string* error_out = nullptr,
                                 const moveit::core::RobotModelConstPtr& robot_model = nullptr);
  
  /**
   * @brief Computes velocity magnitude in configuration space from consecutive waypoints.
   * 
   * Computes the Euclidean distance in joint space divided by time difference.
   * Used when trajectory doesn't have velocity data.
   * 
   * @param positions1 First joint configuration
   * @param positions2 Second joint configuration (consecutive waypoint)
   * @param time_diff Time difference between waypoints in seconds
   * @return Velocity magnitude in configuration space (radians per second)
   */
  static double computeVelocity(const std::vector<double>& positions1,
                                const std::vector<double>& positions2,
                                double time_diff);

  /**
   * @brief Optimizes a trajectory using selected methods (TOTG, Parabolic, or both).
   * 
   * @param trajectory The trajectory to optimize (will be modified in place)
   * @param group_name The planning group name (e.g., "arm")
   * @param use_time_optimal_trajectory_generation Use TimeOptimalTrajectoryGeneration (default: false)
   * @param use_iterative_parabolic Use IterativeParabolicTimeParameterization (default: false)
   * @param robot_model Optional robot model to avoid reloading
   * @return true if at least one optimization method succeeded, false otherwise
   */
  static bool optimizeTrajectory(trajectory_msgs::JointTrajectory& trajectory, 
                                 const std::string& group_name = "arm",
                                 bool use_time_optimal_trajectory_generation = false,
                                 bool use_iterative_parabolic = false,
                                 const moveit::core::RobotModelConstPtr& robot_model = nullptr);
  
  /**
   * @brief Generates a Python matplotlib script for trajectory comparison visualization.
   * 
   * Creates a Python script that plots 4 subplots comparing original and optimized trajectories:
   * 1. Performance metrics (duration, path length)
   * 2. Optimization gains (percentage improvements)
   * 3. Summary comparison table
   * 
   * The function extracts all metrics from the provided PlanMetadata objects, eliminating the need
   * for individual parameter passing and improving code encapsulation.
   * 
   * @param original_plan The original plan containing metrics (duration, path length, etc.)
   * @param optimized_plan The optimized plan containing corresponding metrics
   * @param output_dir Directory where the script and PNG will be saved
   * @return true if script was successfully generated and executed, false otherwise
   */
  static bool generateTrajectoryComparisonPlot(const PlanMetadata& original_plan,
                                               const PlanMetadata& optimized_plan,
                                               const std::string& output_dir);

  /**
   * @brief Validates a single joint configuration against joint limits.
   * 
   * Checks if all joint positions are within their allowed bounds.
   * 
   * @param joint_config Vector of joint positions
   * @param joint_names Names of the joints corresponding to positions in joint_config
   * @param group_name The planning group name for context
   * @param robot_model Optional robot model to avoid reloading
   * @return true if all joints are within limits, false otherwise
   */
  static bool isJointLimitsValid(const std::vector<double>& joint_config,
                                 const std::vector<std::string>& joint_names,
                                 const std::string& group_name = "arm",
                                 const moveit::core::RobotModelConstPtr& robot_model = nullptr);

  /**
   * @brief Validates a single joint configuration for collision with obstacles.
   * 
   * Checks if the robot at a given configuration collides with the planning scene.
   * 
   * @param joint_config Vector of joint positions
   * @param joint_names Names of the joints corresponding to positions in joint_config
   * @param group_name The planning group name
   * @param planning_scene The planning scene for collision checking
   * @param robot_model Optional robot model to avoid reloading
   * @return true if configuration is collision-free, false if in collision
   */
  static bool isCollisionFree(const std::vector<double>& joint_config,
                              const std::vector<std::string>& joint_names,
                              const std::string& group_name = "arm",
                              const planning_scene::PlanningScenePtr& planning_scene = planning_scene::PlanningScenePtr(),
                              const moveit::core::RobotModelConstPtr& robot_model = nullptr);

  /**
   * @brief Validates a path segment between two joint configurations for collisions.
   * 
   * Linearly interpolates between two configurations and checks collision at intermediate points.
   * 
   * @param config1 Starting joint configuration
   * @param config2 Ending joint configuration
   * @param joint_names Names of the joints
   * @param group_name The planning group name
   * @param planning_scene The planning scene for collision checking
   * @param num_samples Number of interpolation samples to check (default: 10)
   * @param robot_model Optional robot model to avoid reloading
   * @return true if entire path segment is collision-free, false if collision found
   */
  static bool isPathCollisionFree(const std::vector<double>& config1,
                                  const std::vector<double>& config2,
                                  const std::vector<std::string>& joint_names,
                                  const std::string& group_name = "arm",
                                  const planning_scene::PlanningScenePtr& planning_scene = planning_scene::PlanningScenePtr(),
                                  int num_samples = 10,
                                  const moveit::core::RobotModelConstPtr& robot_model = nullptr);
};

} // namespace neura_motion_planning_challenge

#endif // NEURA_MOTION_PLANNING_CHALLENGE_VALIDATOR_AND_OPTIMIZER_H
