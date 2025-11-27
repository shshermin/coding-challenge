#ifndef NEURA_MOTION_PLANNING_CHALLENGE_MOTION_PLANNING_H
#define NEURA_MOTION_PLANNING_CHALLENGE_MOTION_PLANNING_H

#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Header.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <neura_motion_planning_challenge/DataStructure/CartMotionPlanningData.h>
#include <neura_motion_planning_challenge/DataStructure/TrajectoryData.h>
#include <neura_motion_planning_challenge/Sampling/SamplingStrategy.h>
#include <neura_motion_planning_challenge/InverseKinematics/IKStrategy.h>

namespace neura_motion_planning_challenge
{

/**
 * @brief Class for motion planning using either joint-space or Cartesian-space approaches.
 *
 * This class provides functionalities for planning robot motion trajectories using various planning
 * algorithms. It encapsulates trajectory data through composition rather than inheritance,
 * ensuring a clean separation between planning logic and data storage.
 *
 * @note Planning algorithms can be selected based on user input. The class manages the underlying
 *       MoveIt interfaces for planning and collision checking.
 */
class MotionPlanning {
  // Getter for display publisher
  // Getter for move group
  public:
    ros::Publisher& getDisplayPublisher() { return display_publisher_; }
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> getMoveGroup() { return move_group_; }
public:
  /**
   * @brief Constructor, initializes the motion planning instance with a ROS node handle.
   *
   * @param nh  A reference to the ROS node handle for communication and configuration.
   */
  MotionPlanning(ros::NodeHandle &nh);
  ~MotionPlanning() = default;

  /**
   * @brief Plans a trajectory to reach the specified joint target positions.
   *
   * This function attempts to determine a feasible and collision-free joint trajectory
   * to reach the given target joint positions. If successful, it returns `true` and
   * populates the internal trajectory plan. Otherwise, it returns `false` and the
   * trajectory plan remains unmodified.
   *
   * @param target_joint_values  The desired target positions for each joint,
   *                            specified as a vector of doubles. The order of the
   *                            values should match the joint order of the robot.
   * @param planned_trajectory   The output trajectory that will be populated with the planned path.
   * @param planner_id           The planner algorithm to use (e.g., "RRTConnect", "RRTstar", "PRM").
   *                            Defaults to empty string, which uses the default planner.
   *
   * @return `true` if a trajectory plan was successfully generated, `false` otherwise.
   *
   * @throws std::runtime_error  If an error occurs during planning, a detailed error
   *                            message is thrown.
   */
  bool planJoint(const std::vector<double> &target_joint_values, moveit_msgs::RobotTrajectoryPtr &planned_trajectory, const std::string &planner_id = "");

  /**
   * @brief Plans a trajectory to reach the specified target pose in Cartesian space.
   *
   * This function attempts to determine a feasible and collision-free joint trajectory
   * that will result in the robot reaching the desired pose specified in the `target_cart_pose` message.
   * If successful, it returns `true` and populates the internal trajectory plan. Otherwise, it returns `false` and the
   * trajectory plan remains unmodified.
   *
   * @param target_cart_pose  The desired target pose of the robot in Cartesian space,
   *                          represented by a `geometry_msgs::PoseStamped` message.
   *                          The pose specifies the position (x, y, z) and orientation (quaternion)
   *                          of the robot's reference frame.
   * @param planned_trajectory The output trajectory that will be populated with the planned path.
   * @param planner_id        The planner algorithm to use (e.g., "RRTConnect", "RRTstar", "PRM").
   *                          Defaults to "RRTConnect" if not specified.
   * @param max_planning_time Maximum time (in seconds) allowed for planning. Defaults to 10.0 seconds.
   *
   * @return `true` if a trajectory plan was successfully generated, `false` otherwise.
   *
   * @throws std::runtime_error  If an error occurs during planning, a detailed error
   *                            message is thrown.
   */
  bool planPose(const geometry_msgs::PoseStamped &target_cart_pose, moveit_msgs::RobotTrajectory &planned_trajectory, const std::string &planner_id = "RRTConnect", double max_planning_time = 10.0);



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
   *               Can pass any SamplingStrategy implementation.
   * @param ik_solver The IK strategy to use for solving joint configurations.
   *                 Default: nullptr, which uses JacobianIKSolver.
   *                 Can pass any IKStrategy implementation.
   *
   * @return A moveit_msgs::RobotTrajectory containing the joint trajectory that follows
   *         the Cartesian path. Returns an empty trajectory on failure.
   *
   * @throws std::exception If setup fails (robot model loading, sampling, IK setup, etc).
   */
  bool planNumericalCartesianPath(
      const CartMotionPlanningData &waypoint_A,
      const CartMotionPlanningData &waypoint_B,
      moveit_msgs::RobotTrajectory &planned_trajectory,
      int num_waypoints = 10,
      bool checkCollisions = false,
      const std::shared_ptr<SamplingStrategy>& sampler = nullptr,
      const std::shared_ptr<IKStrategy>& ik_solver = nullptr);

 

  /**
   * @brief Adds collision objects to the planning environment.
   *
   * This function attempts to add the specified array of collision objects (`collision_object_array`)
   * to the planning environment used for motion planning calculations. If successful, it returns `true`.
   * Otherwise, it returns `false` and sets an error message.
   *
   * @param collision_object_array  A `visualization_msgs::MarkerArray` containing markers
   *                                that represent the collision objects. Each marker should be of type
   *                                `visualization_msgs::Marker::CUBE` or `visualization_msgs::Marker::MESH_RESOURCE`.
   *
   * @return `true` if the collision objects were successfully added, `false` otherwise.
   *
   * @throws std::runtime_error  If an error occurs during collision object addition, a detailed error
   *                            message is thrown.
   */
  bool addCollision(const visualization_msgs::MarkerArray &collision_object_array);

  /**
   * @brief Removes collision objects from the planning environment.
   *
   * This function attempts to remove collision objects from the planning environment.
   * If you provide a specific `collision_object_id`, it removes only that object.
   * If you leave the `collision_object_id` empty (""), it removes all collision objects.
   * If successful, it returns `true`. Otherwise, it returns `false` and sets an error message.
   *
   * @param collision_object_id (Optional) The unique identifier of the collision object to remove.
   *                          Leave empty to remove all collision objects.
   *
   * @return `true` if the collision objects were successfully removed, `false` otherwise.
   *
   * @throws std::runtime_error  If an error occurs during collision object removal, a detailed error
   *                            message is thrown.
   *
   * @note This function assumes that the planning environment is already initialized and accessible.
   *       The specific format of the `collision_object_id` depends on the underlying
   *       motion planning library being used. Consult the documentation for more details.
   */
  bool removeCollision(const std::string &collision_object_id = "");

  /**
   * @brief Retrieves the list of available motion planners for the planning group.
   *
   * This function reads the MoveIt OMPL planning configuration file and extracts
   * all available planner names for the specified planning group.
   *
   * @param config_path Path to the OMPL planning YAML configuration file.
   * @param planning_group Name of the planning group (e.g., "arm", "gripper").
   *
   * @return A vector of strings containing the names of all available planners.
   *
   * @throws std::runtime_error If the configuration file cannot be found, is not a YAML file, or cannot be read.
   */
  std::vector<std::string> getAvailablePlanners(const std::string& config_path, const std::string& planning_group) const;

private:
  /**
   * @brief ROS NodeHandle for subscribing to topics and publishing messages.
   *
   * This node handle is used internally by the class for various ROS communication tasks.
   * It is initialized in the constructor and should not be accessed directly from outside the class.
   */
  /**
   * @brief ROS NodeHandle for subscribing to topics and publishing messages.
   *
   * This node handle is used internally by the class for various ROS communication tasks.
   * It is initialized in the constructor and should not be accessed directly from outside the class.
   */
  ros::NodeHandle nh_;
  
  /**
   * @brief MoveIt MoveGroup interface for planning and execution.
   */
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  
  /**
   * @brief Planning scene interface for managing collision objects.
   */
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  
  /**
   * @brief Publisher for displaying planned trajectories in RViz.
   */
  ros::Publisher display_publisher_;

  /**
   * @brief Current trajectory data (composition, not inheritance).
   *
   * This member variable holds the trajectory data for the motion planning instance.
   * It is used to store and manage the planned trajectories, either in joint space or Cartesian space.
   */
  TrajectoryData current_trajectory_;
};

} // namespace neura_motion_planning_challenge

#endif // NEURA_MOTION_PLANNING_CHALLENGE_MOTION_PLANNING_H