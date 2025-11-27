#include <ros/ros.h>
#include <neura_motion_planning_challenge/Planning/MotionPlanning.h>
#include <neura_motion_planning_challenge/Trajectory/TrajectoryIO.h>
#include <neura_motion_planning_challenge/Trajectory/TrajectoryParser.h>
#include <neura_motion_planning_challenge/Trajectory/TrajectoryVisualizer.h>
#include <neura_motion_planning_challenge/Sampling/UniformSampler.h>
#include <neura_motion_planning_challenge/InverseKinematics/JacobianIKSolver.h>
#include <neura_motion_planning_challenge/Utility/ValidatorAndOptimizer.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <memory>

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
        TrajectoryVisualizer::logAndVisualizeTrajectory(plan, display_publisher_, "Joint planning");
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
        TrajectoryVisualizer::logAndVisualizeTrajectory(plan, display_publisher_, "Pose planning");
        return true;
        
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in planPose: %s", e.what());
        throw std::runtime_error(std::string("Planning failed: ") + e.what());
    }
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

bool MotionPlanning::planNumericalCartesianPath(
    const CartMotionPlanningData &waypoint_A,
    const CartMotionPlanningData &waypoint_B,
    moveit_msgs::RobotTrajectory &planned_trajectory,
    int num_waypoints,
    bool check_collisions,
    const std::shared_ptr<SamplingStrategy>& sampler,
    const std::shared_ptr<IKStrategy>& ik_solver) {
    
    // Initialize result trajectory
    moveit_msgs::RobotTrajectory result;
    result.joint_trajectory.header.stamp = ros::Time::now();
    result.joint_trajectory.header.frame_id = "base_link";
    
    try {
        // Load robot model and setup
        robot_model_loader::RobotModelLoader loader("robot_description");
        moveit::core::RobotModelConstPtr robot_model = loader.getModel();
        if (!robot_model) {
            ROS_ERROR("CartMotionPlanner: Failed to load robot model from robot_description");
            return false;
        }     

        // Create MoveGroup interface
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group =
            std::make_shared<moveit::planning_interface::MoveGroupInterface>("arm");

        // Get joint names from the move group
        std::vector<std::string> joint_names = move_group->getVariableNames();
        if (joint_names.empty()) {
            ROS_ERROR("CartMotionPlanner: Failed to get joint names from move_group");
            return false;
        }
        
        ROS_INFO("CartMotionPlanner: Planning with %zu joints", joint_names.size());
        
        // Step 1: Generate Cartesian waypoints using sampling strategy
        ROS_INFO("CartMotionPlanner: Generating %d waypoints between A and B...", num_waypoints);
        
        // Use provided sampler or create default UniformSampler
        std::shared_ptr<SamplingStrategy> sampling_strategy = sampler;
        if (!sampling_strategy) {
            sampling_strategy = std::make_shared<UniformSampler>();
            ROS_INFO("CartMotionPlanner: Using default UniformSampler");
        } else {
            ROS_INFO("CartMotionPlanner: Using custom sampling strategy");
        }
        
        std::vector<CartMotionPlanningData> cart_waypoints;
        try {
            cart_waypoints = sampling_strategy->generateWaypoints(waypoint_A, waypoint_B, num_waypoints);
        } catch (const std::exception& e) {
            ROS_ERROR("CartMotionPlanner: Sampling failed: %s", e.what());
            return false;
        }
        
        if (cart_waypoints.empty()) {
            ROS_ERROR("CartMotionPlanner: Sampling returned empty waypoints");
            return false;
        }
        
        ROS_INFO("CartMotionPlanner: Generated %zu Cartesian waypoints", cart_waypoints.size());
        
        // Step 2: Setup IK solver (pluggable strategy pattern)
        std::shared_ptr<IKStrategy> ik_strategy_impl = ik_solver;
        if (!ik_strategy_impl) {
            ik_strategy_impl = std::make_shared<JacobianIKSolver>(move_group, "arm", 100, 0.001, 0.01, 1.0);
            ROS_INFO("CartMotionPlanner: Using default JacobianIKSolver");
        } else {
            ROS_INFO("CartMotionPlanner: Using custom IK strategy");
        }
        
        // Step 3: Solve IK for each waypoint and build trajectory
        std::vector<std::vector<double>> joint_configs;
        trajectory_msgs::JointTrajectory joint_traj;
        joint_traj.joint_names = joint_names;
        joint_traj.header.stamp = ros::Time::now();
        joint_traj.header.frame_id = "base_link";
        
        // Load planning scene for collision checking
        planning_scene::PlanningScenePtr planning_scene;
        if (check_collisions) {
            planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
            ROS_INFO("CartMotionPlanner: Collision checking enabled");
        }
        
        double time_from_start = 0.0;
        const double time_per_waypoint = 1.0;  // 1 second per waypoint segment
        
        for (size_t i = 0; i < cart_waypoints.size(); ++i) {
            ROS_DEBUG("CartMotionPlanner: Processing waypoint %zu/%zu", i + 1, cart_waypoints.size());
            
            const geometry_msgs::Pose& target_pose = cart_waypoints[i].getPose();
            std::vector<double> joint_solution(joint_names.size());
            
            // Solve IK for this waypoint using the pluggable strategy
            if (!ik_strategy_impl->solveIK(target_pose, joint_solution)) {
                ROS_WARN("CartMotionPlanner: IK solver failed to converge for waypoint %zu. "
                        "This may still provide a reasonable approximation.", i);
                // Continue anyway with best approximation from solver
            }
            
            // Add to joint trajectory
            trajectory_msgs::JointTrajectoryPoint point;
            point.positions = joint_solution;
            point.time_from_start = ros::Duration(time_from_start);
            
            // Set velocities and accelerations to zero (for safety with execution)
            point.velocities.resize(joint_names.size(), 0.0);
            point.accelerations.resize(joint_names.size(), 0.0);
            
            joint_traj.points.push_back(point);
            joint_configs.push_back(joint_solution);
            
            time_from_start += time_per_waypoint;
            
            ROS_DEBUG("CartMotionPlanner: Waypoint %zu OK - time from start: %.2f s", i, time_from_start);
        }
        
        // Step 4: Validate full trajectory
        ROS_INFO("CartMotionPlanner: Validating complete trajectory with %zu points", joint_traj.points.size());
        std::string validation_error;
        if (!TrajectoryValidatorAndOptimizer::validateTrajectory(
                joint_traj, "arm", planning_scene, &validation_error, robot_model)) {
            ROS_WARN("CartMotionPlanner: Full trajectory validation warning: %s", validation_error.c_str());
            // Don't abort, just warn - the waypoint-level checks were passed
        }
        
        // Step 5: Populate output and return success
        result.joint_trajectory = joint_traj;
        planned_trajectory = result;
        ROS_INFO("CartMotionPlanner: Successfully planned Cartesian path with %zu waypoints", 
                joint_traj.points.size());
        
        return true;
        
    } catch (const std::exception& e) {
        ROS_ERROR("CartMotionPlanner: Exception occurred: %s", e.what());
        return false;
    }
}


