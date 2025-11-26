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
#include <neura_motion_planning_challenge/CartMotionPlanningData.h>
#include <neura_motion_planning_challenge/TrajectoryData.h>

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
   * @brief Plans a robot trajectory consisting of multiple waypoints in Cartesian space.
   *
   * This function attempts to determine a feasible and collision-free joint trajectory
   * that follows the sequence of waypoints provided in the `cart_waypoints` vector.
   * If successful, it returns a reference to the generated `moveit_msgs::RobotTrajectory` and populates it with the planned path.
   * Otherwise, an exception is thrown.
   *
   * @param cart_waypoints  A vector of `CartMotionPlanningData` structs,
   *                        where each struct contains:
   *
   * @return A reference to the generated `moveit_msgs::RobotTrajectory`.
   *
   * @throws std::runtime_error  If an error occurs during planning, a detailed error
   *                            message is thrown.
   */
  moveit_msgs::RobotTrajectory &planCartesian(const std::vector<CartMotionPlanningData> &cart_waypoints);

  /**
   * @brief Exports a robot trajectory to a file.
   *
   * This function writes the provided joint trajectory (`joint_trajectory`) and an optional set of corresponding Cartesian waypoints (`cart_pose_array`) to a file specified by `filepath`.
   * If successful, it returns `true`. Otherwise, it returns `false` and sets an error message.
   *
   * @param filepath  The path to the file where the trajectory will be saved.
   * @param joint_trajectory  The planned joint trajectory to be exported, represented as a `trajectory_msgs::JointTrajectory` message.
   * @param cart_pose_array  (Optional) A vector of `CartMotionPlanningData` structs, corresponding to the waypoints of the joint trajectory.
   *                        If provided, these waypoints will be included in the output file for additional reference.
   *
   * @return `true` if the trajectory was successfully exported, `false` otherwise.
   *
   * @throws std::runtime_error  If an error occurs during file writing, a detailed error
   *                            message is thrown.
   */
  bool exportTrajectoryToFile(const std::string &filepath, const trajectory_msgs::JointTrajectory &joint_trajectory, const std::vector<CartMotionPlanningData> &cart_pose_array);

  /**
   * @brief Imports a robot trajectory from a file and returns it as a TrajectoryData object.
   *
   * This function attempts to read and parse a robot trajectory from the specified file.
   * If successful, it returns a `TrajectoryData` object containing the imported data.
   * Otherwise, it throws an exception indicating the error.
   *
   * @param filepath  The path to the file containing the trajectory data.
   *
   * @return A `TrajectoryData` object holding the imported trajectory information.
   *
   * @throws std::runtime_error  If an error occurs during file reading or parsing,
   *                             a detailed error message is thrown.
   *
   * @note The specific format of the input file may vary depending on the implementation.
   *       Consult the class documentation for supported formats and the structure of the
   *       returned `TrajectoryData` object.
   *       Additionally, this function might require appropriate permissions to read the
   *       specified file.
   */
  TrajectoryData importTrajectoryFromFile(const std::string &filepath);

  /**
   * @brief Visualizes a planned robot trajectory in RViz.
   *
   * This function attempts to display the given joint trajectory message (`joint_trajectory`)
   * within the RViz environment. If successful, it returns `true`. Otherwise, it returns `false`
   * and sets an error message.
   *
   * @param joint_trajectory  A pointer to a `moveit_msgs::RobotTrajectory` message containing the joint values
   *                           and timing information for the planned trajectory.
   *
   * @return `true` if the trajectory was successfully visualized, `false` otherwise.
   *
   * @throws std::runtime_error  If an error occurs during visualization, a detailed error message is thrown.
   *
   * @note This function assumes that RViz is running and properly configured with the robot model.
   *       Additionally, it might require specific plugins or settings to function correctly.
   *       Consult the documentation for more details.
   */
  bool visualizeJointTrajectory(const moveit_msgs::RobotTrajectoryPtr &joint_trajectory);

  /**
   * @brief Visualizes a sequence of Cartesian waypoints in RViz.
   *
   * This function attempts to display each waypoint from the provided vector `cart_pose_array`
   * as a marker in RViz. If successful, it returns `true`. Otherwise, it returns `false` and sets an error message.
   *
   * @param cart_pose_array  A vector of `CartMotionPlanningData` structs, where each struct represents a desired
   *                          Cartesian pose of the robot.
   *
   * @return `true` if the waypoints were successfully visualized, `false` otherwise.
   *
   * @throws std::runtime_error  If an error occurs during visualization, a detailed error message is thrown.
   *
   * @note This function assumes that RViz is running and properly configured with the robot model.
   *       Additionally, it might require specific markers plugins or settings to function correctly.
   *       Consult the documentation for more details.
   */
  bool visualizeCartTrajectory(const std::vector<CartMotionPlanningData> &cart_pose_array);

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
   * @brief Helper method to visualize and log trajectory details.
   * @param plan The MoveIt Plan object containing the trajectory.
   * @param method_name Name of the calling method (for logging purposes).
   */
  void logAndVisualizeTrajectory(const moveit::planning_interface::MoveGroupInterface::Plan& plan, const std::string& method_name);

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