#include <ros/ros.h>
#include <neura_motion_planning_challenge/Planning/motion_planning.h>
#include <neura_motion_planning_challenge/Trajectory/TrajectoryIO.h>
#include <neura_motion_planning_challenge/Trajectory/TrajectoryParser.h>
#include <neura_motion_planning_challenge/Trajectory/TrajectoryVisualizer.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <fstream>
#include <sstream>
#include <algorithm>

using namespace neura_motion_planning_challenge;

// MotionPlanning class implementation

/**
 * @brief Constructor: Initializes MoveIt interfaces and RViz publisher.
 * @param nh Reference to the ROS node handle for communication.
 */
MotionPlanning::MotionPlanning(ros::NodeHandle& nh) : nh_(nh)
{
    // Initialize MoveGroup interface
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>("arm");
    
    if (!move_group_) {
        throw std::runtime_error("Failed to initialize MoveGroupInterface");
    }
    
    // Initialize PlanningSceneInterface for collision objects
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    
    if (!planning_scene_interface_) {
        throw std::runtime_error("Failed to initialize PlanningSceneInterface");
    }
    
    // Create publisher for displaying trajectories in RViz
    display_publisher_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    
    // Wait for publisher to establish connections
    ros::Duration(1.0).sleep();
    
    ROS_INFO("MotionPlanning class initialized with planning group: arm");
    ROS_INFO("Display publisher has %d subscribers", display_publisher_.getNumSubscribers());
}


/**
 * @brief Plans a trajectory in joint space to reach target joint positions.
 * @param target_joint_values Target joint positions to reach.
 * @param planned_trajectory Output trajectory that will be populated with the planned path.
 * @param planner_id Planner algorithm to use (defaults to empty string for default planner).
 * @return true if planning succeeded, false otherwise.
 * @throws std::runtime_error If an error occurs during planning.
 */
bool MotionPlanning::planJoint(const std::vector<double>& target_joint_values, moveit_msgs::RobotTrajectoryPtr& planned_trajectory, const std::string& planner_id)
{
    try {
        // Validate input
        if (target_joint_values.empty()) {
            ROS_ERROR("Target joint values cannot be empty");
            return false;
        }
        
        // Set planner if specified
        if (!planner_id.empty()) {
            move_group_->setPlannerId(planner_id);
        }
        
        // Set target and plan
        move_group_->setJointValueTarget(target_joint_values);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        if (!success) {
            ROS_WARN("Joint planning failed");
            return false;
        }
        
        // Store result
        if (!planned_trajectory) {
            planned_trajectory.reset(new moveit_msgs::RobotTrajectory());
        }
        *planned_trajectory = plan.trajectory_;
        
        // Visualize and log
        logAndVisualizeTrajectory(plan, "Joint planning");
        return true;
        
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in planJoint: %s", e.what());
        throw std::runtime_error(std::string("Planning failed: ") + e.what());
    }
}


/**
 * @brief Plans a trajectory to reach a target Cartesian pose.
 * @param target_cart_pose Target pose in Cartesian space.
 * @param planned_trajectory Output trajectory that will be populated with the planned path.
 * @param planner_id Planner algorithm to use (defaults to "RRTConnect").
 * @param max_planning_time Maximum planning time in seconds (defaults to 10.0).
 * @return true if planning succeeded, false otherwise.
 * @throws std::runtime_error If an error occurs during planning.
 */
bool MotionPlanning::planPose(const geometry_msgs::PoseStamped& target_cart_pose, moveit_msgs::RobotTrajectory& planned_trajectory, const std::string& planner_id, double max_planning_time)
{
    try {
        // Validate input
        if (max_planning_time <= 0.0) {
            ROS_ERROR("Planning time must be positive (got %.1f seconds)", max_planning_time);
            return false;
        }
        
        // Set planner if specified
        if (!planner_id.empty()) {
            move_group_->setPlannerId(planner_id);
            ROS_DEBUG("Using planner: %s", planner_id.c_str());
        }
        
        // Set planning time limit
        move_group_->setPlanningTime(max_planning_time);
        ROS_DEBUG("Planning time limit: %.1f seconds", max_planning_time);
        
        // Set the target pose
        bool pose_set = move_group_->setPoseTarget(target_cart_pose);
        if (!pose_set) {
            ROS_ERROR("Failed to set target pose - pose may be unreachable or invalid");
            return false;
        }
        ROS_DEBUG("Target pose set successfully");
        
        // Plan the trajectory
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        ROS_DEBUG("Planning...");
        moveit::planning_interface::MoveItErrorCode error_code = move_group_->plan(plan);
        
        bool success = (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        if (!success) {
            ROS_WARN("Pose planning failed with error code: %d", error_code.val);
            return false;
        }
        
        // Store result
        planned_trajectory = plan.trajectory_;
        
        // Visualize and log
        logAndVisualizeTrajectory(plan, "Pose planning");
        return true;
        
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in planPose: %s", e.what());
        throw std::runtime_error(std::string("Planning failed: ") + e.what());
    }
}


/**
 * @brief Plans a trajectory through multiple Cartesian waypoints (stub - not yet implemented).
 * @param cart_waypoints Vector of Cartesian waypoints to traverse.
 * @return Reference to the generated robot trajectory.
 * @throws std::runtime_error Always throws - method is not yet implemented.
 */
moveit_msgs::RobotTrajectory& MotionPlanning::planCartesian(const std::vector<CartMotionPlanningData>& cart_waypoints)
{
    throw std::runtime_error("planCartesian() is not yet implemented");
}

/**
 * @brief Exports a trajectory (joint and Cartesian) to a file in YAML format.
 * @param filepath The destination file path where the trajectory will be saved.
 * @param joint_trajectory The joint trajectory data to export.
 * @param cart_pose_array Vector of Cartesian waypoints to export.
 * @return true if export was successful, false otherwise.
 */
bool MotionPlanning::exportTrajectoryToFile(const std::string& filepath, const trajectory_msgs::JointTrajectory& joint_trajectory, const std::vector<CartMotionPlanningData>& cart_pose_array)
{
    try {
        std::ofstream file(filepath);
        if (!file.is_open()) {
            ROS_ERROR("Failed to open file for writing: %s", filepath.c_str());
            return false;
        }
        
        // Write header
        file << "# Robot Trajectory Export\n"
             << "# Generated at: " << ros::Time::now() << "\n"
             << "# File format: YAML-like structure\n"
             << "---\n\n";
        
        // Export joint trajectory and Cartesian poses using helper methods
        neura_motion_planning_challenge::TrajectoryIO::writeJointTrajectory(file, joint_trajectory);
        neura_motion_planning_challenge::TrajectoryIO::writeCartesianPoses(file, cart_pose_array);
        
        file.close();
        ROS_INFO("Successfully exported trajectory to: %s", filepath.c_str());
        ROS_INFO("  Joint waypoints: %lu", joint_trajectory.points.size());
        ROS_INFO("  Cartesian poses: %lu", cart_pose_array.size());
        return true;
        
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in exportTrajectoryToFile: %s", e.what());
        return false;
    }
}



/**
 * @brief Imports a trajectory from a file in JSON or YAML format.
 * @param filepath The source file path containing the trajectory data.
 * @return TrajectoryData object populated with the imported trajectory.
 * @throws std::runtime_error If the file cannot be opened or parsing fails.
 */
TrajectoryData MotionPlanning::importTrajectoryFromFile(const std::string& filepath)
{
    TrajectoryData trajectory_data;
    
    try {
        trajectory_msgs::JointTrajectory& joint_traj = trajectory_data.getJointTrajectory();
        
        // Try JSON format first (check file extension)
        if (filepath.length() >= 5 && filepath.substr(filepath.length() - 5) == ".json") {
            ROS_INFO("Detected JSON format, parsing...");
            if (neura_motion_planning_challenge::TrajectoryParser::parseJsonTrajectory(filepath, joint_traj)) {
                ROS_INFO("Successfully imported trajectory from: %s", filepath.c_str());
                ROS_INFO("  Joint waypoints loaded: %lu", joint_traj.points.size());
                return trajectory_data;
            } else {
                ROS_WARN("JSON parsing failed, trying YAML format...");
            }
        }
        
        // Fall back to YAML-like format
        std::ifstream file(filepath);
        if (!file.is_open()) {
            ROS_ERROR("Failed to open file for reading: %s", filepath.c_str());
            throw std::runtime_error("Cannot open trajectory file");
        }
        
        std::string line;
        bool in_joint_section = false;
        bool in_cart_section = false;
        
        while (std::getline(file, line)) {
            // Skip comments and empty lines
            if (line.empty() || line[0] == '#' || line.find("---") == 0) {
                continue;
            }
            
            // Detect sections
            if (line.find("joint_trajectory:") != std::string::npos) {
                in_joint_section = true;
                in_cart_section = false;
                continue;
            }
            if (line.find("cartesian_poses:") != std::string::npos) {
                in_joint_section = false;
                in_cart_section = true;
                continue;
            }
            
            // Parse joint trajectory section
            if (in_joint_section) {
                if (line.find("joint_names:") != std::string::npos) {
                    neura_motion_planning_challenge::TrajectoryParser::parseJointNames(line, joint_traj.joint_names);
                }
                else if (line.find("- index:") != std::string::npos) {
                    neura_motion_planning_challenge::TrajectoryParser::parseWaypoint(file, line, joint_traj);
                }
            }
        }
        
        file.close();
        ROS_INFO("Successfully imported trajectory from: %s", filepath.c_str());
        ROS_INFO("  Joint waypoints loaded: %lu", joint_traj.points.size());
        
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in importTrajectoryFromFile: %s", e.what());
        throw;
    }
    
    return trajectory_data;
}

/**
 * @brief Visualizes a joint trajectory in RViz.
 * @param joint_trajectory The trajectory to visualize.
 * @return true if visualization succeeded, false otherwise.
 */
bool MotionPlanning::visualizeJointTrajectory(const moveit_msgs::RobotTrajectoryPtr& joint_trajectory)
{
    if (!joint_trajectory) {
        ROS_ERROR("Cannot visualize null trajectory");
        return false;
    }
    
    try {
        moveit_msgs::DisplayTrajectory display_trajectory;
        
        // Get current robot state as the starting state
        robot_state::RobotStatePtr current_state = move_group_->getCurrentState(3.0);
        if (current_state) {
            moveit::core::robotStateToRobotStateMsg(*current_state, display_trajectory.trajectory_start);
        }
        
        display_trajectory.trajectory.push_back(*joint_trajectory);
        display_trajectory.model_id = "mira";
        
        // Publish multiple times to ensure RViz receives it
        for (int i = 0; i < 5; i++) {
            display_publisher_.publish(display_trajectory);
            ros::Duration(0.1).sleep();
            ros::spinOnce();
        }
        
        ROS_INFO("Joint trajectory visualized in RViz (%d subscribers)", display_publisher_.getNumSubscribers());
        return true;
        
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in visualizeJointTrajectory: %s", e.what());
        return false;
    }
}

/**
 * @brief Visualizes a sequence of Cartesian waypoints in RViz (stub - not yet implemented).
 * @param cart_pose_array Vector of Cartesian waypoints to visualize.
 * @return true if visualization succeeded, false otherwise.
 */
bool MotionPlanning::visualizeCartTrajectory(const std::vector<CartMotionPlanningData>& cart_pose_array)
{
    // Not yet implemented
    return false;
}

/**
 * @brief Adds collision objects to the planning scene from a marker array.
 * @param collision_object_array MarkerArray containing collision objects to add.
 * @return true if all objects were added successfully, false otherwise.
 */
bool MotionPlanning::addCollision(const visualization_msgs::MarkerArray& collision_object_array)
{
    if (collision_object_array.markers.empty()) {
        ROS_WARN("No collision objects to add - marker array is empty");
        return false;
    }
    
    try {
        std::vector<moveit_msgs::CollisionObject> collision_objects;
        
        // Convert each marker to a CollisionObject
        for (const auto& marker : collision_object_array.markers) {
            moveit_msgs::CollisionObject collision_object;
            collision_object.header = marker.header;
            collision_object.id = marker.ns.empty() ? ("object_" + std::to_string(marker.id)) : marker.ns;
            
            // Define the primitive shape based on marker type
            shape_msgs::SolidPrimitive primitive;
            
            if (marker.type == visualization_msgs::Marker::CUBE) {
                primitive.type = primitive.BOX;
                primitive.dimensions.resize(3);
                primitive.dimensions[0] = marker.scale.x;
                primitive.dimensions[1] = marker.scale.y;
                primitive.dimensions[2] = marker.scale.z;
            }
            else if (marker.type == visualization_msgs::Marker::SPHERE) {
                primitive.type = primitive.SPHERE;
                primitive.dimensions.resize(1);
                primitive.dimensions[0] = marker.scale.x; // radius
            }
            else if (marker.type == visualization_msgs::Marker::CYLINDER) {
                primitive.type = primitive.CYLINDER;
                primitive.dimensions.resize(2);
                primitive.dimensions[0] = marker.scale.z; // height
                primitive.dimensions[1] = marker.scale.x; // radius
            }
            else {
                ROS_WARN("Unsupported marker type %d for object %s - skipping", marker.type, collision_object.id.c_str());
                continue;
            }
            
            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(marker.pose);
            collision_object.operation = collision_object.ADD;
            
            collision_objects.push_back(collision_object);
        }
        
        if (collision_objects.empty()) {
            ROS_WARN("No valid collision objects created from markers");
            return false;
        }
        
        // Add objects to planning scene
        planning_scene_interface_->addCollisionObjects(collision_objects);
        
        // Wait for the planning scene to update
        ros::Duration(0.5).sleep();
        
        ROS_INFO("Successfully added %lu collision object(s) to the planning scene:", collision_objects.size());
        for (const auto& obj : collision_objects) {
            ROS_INFO("  - %s", obj.id.c_str());
        }
        
        return true;
        
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in addCollision: %s", e.what());
        return false;
    }
}

/**
 * @brief Removes collision objects from the planning scene by ID or removes all if ID is empty.
 * @param collision_object_id ID of the collision object to remove. Use empty string to remove all objects.
 * @return true if removal succeeded, false otherwise.
 */
bool MotionPlanning::removeCollision(const std::string& collision_object_id)
{
    try {
        std::vector<std::string> object_ids;
        
        // If empty string is provided, remove all collision objects
        if (collision_object_id.empty()) {
            ROS_INFO("Removing all collision objects from the planning scene...");
            planning_scene_interface_->removeCollisionObjects(object_ids);  // Empty vector removes all
        } else {
            // Remove specific collision object by ID
            object_ids.push_back(collision_object_id);
            planning_scene_interface_->removeCollisionObjects(object_ids);
            ROS_INFO("Attempting to remove collision object: %s", collision_object_id.c_str());
        }
        
        // Wait for the planning scene to update
        ros::Duration(0.5).sleep();
        
        if (collision_object_id.empty()) {
            ROS_INFO("Successfully removed all collision objects from the planning scene");
        } else {
            ROS_INFO("Successfully removed collision object: %s", collision_object_id.c_str());
        }
        
        return true;
        
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in removeCollision: %s", e.what());
        return false;
    }
}


// Private helper method to visualize and log trajectory
void MotionPlanning::logAndVisualizeTrajectory(const moveit::planning_interface::MoveGroupInterface::Plan& plan, const std::string& method_name)
{
    neura_motion_planning_challenge::TrajectoryVisualizer::publishTrajectoryToRViz(plan, display_publisher_);
    neura_motion_planning_challenge::TrajectoryVisualizer::logTrajectoryDetails(plan.trajectory_.joint_trajectory);
    ROS_INFO("%s succeeded with %lu waypoints", method_name.c_str(), plan.trajectory_.joint_trajectory.points.size());
}


/**
 * @brief Retrieves list of available planners from MoveIt YAML configuration file.
 * @param config_path Path to the OMPL configuration file (.yaml).
 * @param planning_group Name of the planning group to query.
 * @return Vector of available planner names.
 * @throws std::runtime_error If file cannot be opened or format is invalid.
 */
std::vector<std::string> MotionPlanning::getAvailablePlanners(const std::string& config_path, const std::string& planning_group) const
{
    std::vector<std::string> planners;
    
    // Validate that the file has .yaml extension
    if (config_path.length() < 5 || config_path.substr(config_path.length() - 5) != ".yaml") {
        ROS_ERROR("Invalid file format: %s (expected .yaml file)", config_path.c_str());
        throw std::runtime_error("Configuration file must be a .yaml file");
    }
    
    std::ifstream file(config_path);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open OMPL planning config file: %s", config_path.c_str());
        throw std::runtime_error("Cannot read planner configuration file");
    }
    
    // Parse the YAML file to find planners for the specified planning group
    std::string line;
    bool in_group_section = false;
    bool in_planner_configs = false;
    std::string group_marker = planning_group + ":";
    
    while (std::getline(file, line)) {
        // Check if we're in the specified planning group section
        if (line.find(group_marker) == 0) {
            in_group_section = true;
            continue;
        }
        
        // Exit group section when we hit another top-level section
        if (in_group_section && !line.empty() && line[0] != ' ') {
            break;
        }
        
        // Look for planner_configs subsection
        if (in_group_section && line.find("  planner_configs:") != std::string::npos) {
            in_planner_configs = true;
            continue;
        }
        
        // Parse planner names (lines starting with "    - ")
        if (in_planner_configs && line.find("    - ") != std::string::npos) {
            // Extract planner name
            size_t start = line.find("- ") + 2;
            std::string planner_name = line.substr(start);
            // Trim whitespace
            planner_name.erase(planner_name.find_last_not_of(" \t\n\r") + 1);
            planners.push_back(planner_name);
        }
        
        // Exit planner_configs when we hit another subsection
        if (in_planner_configs && !line.empty() && line.find("    - ") == std::string::npos && line.find("  ") == 0 && line[2] != ' ') {
            break;
        }
    }
    
    file.close();
    
    if (planners.empty()) {
        ROS_WARN("No planners found for planning group '%s' in configuration file", planning_group.c_str());
    } else {
        ROS_INFO("Found %lu available planners for planning group '%s'", planners.size(), planning_group.c_str());
    }
    
    return planners;
}


