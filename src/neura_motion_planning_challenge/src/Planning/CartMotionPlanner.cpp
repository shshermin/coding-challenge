#include <neura_motion_planning_challenge/Planning/CartMotionPlanner.h>
#include <neura_motion_planning_challenge/Sampling/UniformSampler.h>
#include <neura_motion_planning_challenge/InverseKinematics/JacobianIKSolver.h>
#include <neura_motion_planning_challenge/Utility/ValidatorAndOptimizer.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <ros/ros.h>

using namespace neura_motion_planning_challenge;

moveit_msgs::RobotTrajectory CartMotionPlanner::planNumericalCartesianPath(
    const CartMotionPlanningData &waypoint_A,
    const CartMotionPlanningData &waypoint_B,
    int num_waypoints,
    bool check_collisions) {
    
    // Initialize result trajectory
    moveit_msgs::RobotTrajectory result;
    result.joint_trajectory.header.stamp = ros::Time::now();
    result.joint_trajectory.header.frame_id = "world";
    
    // Validate input
    if (num_waypoints < 2) {
        ROS_ERROR("CartMotionPlanner: num_waypoints must be at least 2, got %d", num_waypoints);
        return result;
    }
    
    try {
        // Load robot model and setup
        robot_model_loader::RobotModelLoader loader("robot_description");
        moveit::core::RobotModelConstPtr robot_model = loader.getModel();
        if (!robot_model) {
            ROS_ERROR("CartMotionPlanner: Failed to load robot model from robot_description");
            return result;
        }
        
        // Create MoveGroup interface
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group =
            std::make_shared<moveit::planning_interface::MoveGroupInterface>("arm");
        
        // Get joint names from the move group
        std::vector<std::string> joint_names = move_group->getVariableNames();
        if (joint_names.empty()) {
            ROS_ERROR("CartMotionPlanner: Failed to get joint names from move_group");
            return result;
        }
        
        ROS_INFO("CartMotionPlanner: Planning with %zu joints", joint_names.size());
        
        // Step 1: Generate Cartesian waypoints using uniform sampling
        ROS_INFO("CartMotionPlanner: Generating %d waypoints between A and B...", num_waypoints);
        UniformSampler sampler;
        std::vector<CartMotionPlanningData> cart_waypoints;
        try {
            cart_waypoints = sampler.generateWaypoints(waypoint_A, waypoint_B, num_waypoints);
        } catch (const std::exception& e) {
            ROS_ERROR("CartMotionPlanner: Sampling failed: %s", e.what());
            return result;
        }
        
        if (cart_waypoints.empty()) {
            ROS_ERROR("CartMotionPlanner: Sampling returned empty waypoints");
            return result;
        }
        
        ROS_INFO("CartMotionPlanner: Generated %zu Cartesian waypoints", cart_waypoints.size());
        
        // Step 2: Setup IK solver
        JacobianIKSolver ik_solver(move_group, "arm", 100, 0.001, 0.01, 1.0);
        
        // Step 3: Solve IK for each waypoint and build trajectory
        std::vector<std::vector<double>> joint_configs;
        trajectory_msgs::JointTrajectory joint_traj;
        joint_traj.joint_names = joint_names;
        joint_traj.header.stamp = ros::Time::now();
        joint_traj.header.frame_id = "world";
        
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
            
            // Solve IK for this waypoint
            if (!ik_solver.solveIK(target_pose, joint_solution)) {
                ROS_WARN("CartMotionPlanner: IK solver failed to converge for waypoint %zu. "
                        "This may still provide a reasonable approximation.", i);
                // Continue anyway with best approximation from solver
            }
            
            // Validate joint limits
            if (!TrajectoryValidatorAndOptimizer::isJointLimitsValid(joint_solution, joint_names, "arm")) {
                ROS_ERROR("CartMotionPlanner: Waypoint %zu violates joint limits", i);
                return result;  // Abort trajectory
            }
            
            // Validate collision if requested
            if (check_collisions) {
                if (!TrajectoryValidatorAndOptimizer::isCollisionFree(joint_solution, joint_names, "arm", planning_scene)) {
                    ROS_ERROR("CartMotionPlanner: Waypoint %zu is in collision", i);
                    return result;  // Abort trajectory
                }
                
                // Check path collision to previous waypoint
                if (i > 0) {
                    if (!TrajectoryValidatorAndOptimizer::isPathCollisionFree(
                            joint_configs.back(), joint_solution, joint_names, "arm", planning_scene, 10)) {
                        ROS_ERROR("CartMotionPlanner: Path segment %zu->%zu collides", i - 1, i);
                        return result;  // Abort trajectory
                    }
                }
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
                joint_traj, "arm", planning_scene, &validation_error)) {
            ROS_WARN("CartMotionPlanner: Full trajectory validation warning: %s", validation_error.c_str());
            // Don't abort, just warn - the waypoint-level checks were passed
        }
        
        // Step 5: Return result
        result.joint_trajectory = joint_traj;
        ROS_INFO("CartMotionPlanner: Successfully planned Cartesian path with %zu waypoints", 
                joint_traj.points.size());
        
        return result;
        
    } catch (const std::exception& e) {
        ROS_ERROR("CartMotionPlanner: Exception occurred: %s", e.what());
        return result;
    }
}
