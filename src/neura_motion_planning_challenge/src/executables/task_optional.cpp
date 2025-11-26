// Optional Task: Cartesian path planning using numerical approach
// Plans a straight-line path from pose A to pose B without using MoveIt planners

#include <ros/ros.h>
#include <neura_motion_planning_challenge/motion_planning.h>
#include <neura_motion_planning_challenge/TrajectoryValidatorAndOptimizer.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/planning_scene/planning_scene.h>
#include <cmath>

using namespace neura_motion_planning_challenge;

// Helper: Interpolate between two poses with linear position and SLERP orientation
std::vector<geometry_msgs::Pose> interpolateCartesianPath(
    const geometry_msgs::Pose& start,
    const geometry_msgs::Pose& goal,
    double step_size = 0.01)  // 1cm steps
{
    std::vector<geometry_msgs::Pose> waypoints;
    
    // Calculate Euclidean distance
    double dx = goal.position.x - start.position.x;
    double dy = goal.position.y - start.position.y;
    double dz = goal.position.z - start.position.z;
    double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
    
    // Calculate number of waypoints needed
    int num_steps = std::max(2, static_cast<int>(std::ceil(distance / step_size)));
    
    ROS_INFO("Interpolating Cartesian path:");
    ROS_INFO("  Distance: %.3f m", distance);
    ROS_INFO("  Number of waypoints: %d", num_steps);
    
    // Convert quaternions to tf2 for SLERP
    tf2::Quaternion q_start(start.orientation.x, start.orientation.y, 
                             start.orientation.z, start.orientation.w);
    tf2::Quaternion q_goal(goal.orientation.x, goal.orientation.y, 
                            goal.orientation.z, goal.orientation.w);
    
    // Interpolate
    for (int i = 0; i <= num_steps; i++) {
        double t = static_cast<double>(i) / num_steps;
        
        geometry_msgs::Pose waypoint;
        
        // Linear interpolation for position
        waypoint.position.x = start.position.x + t * dx;
        waypoint.position.y = start.position.y + t * dy;
        waypoint.position.z = start.position.z + t * dz;
        
        // SLERP for orientation
        tf2::Quaternion q_interp = q_start.slerp(q_goal, t);
        waypoint.orientation.x = q_interp.x();
        waypoint.orientation.y = q_interp.y();
        waypoint.orientation.z = q_interp.z();
        waypoint.orientation.w = q_interp.w();
        
        waypoints.push_back(waypoint);
    }
    
    return waypoints;
}

// Helper: Solve IK for a Cartesian pose using MoveIt's IK solver (numerical method)
bool solveIK(const moveit::planning_interface::MoveGroupInterface& move_group,
             const geometry_msgs::Pose& target_pose,
             std::vector<double>& joint_solution)
{
    // Get current robot state
    moveit::core::RobotStatePtr robot_state = move_group.getCurrentState();
    const robot_model::JointModelGroup* joint_model_group = 
        robot_state->getJointModelGroup(move_group.getName());
    
    // Attempt to solve IK
    bool found_ik = robot_state->setFromIK(joint_model_group, target_pose, 10, 0.1);
    
    if (found_ik) {
        robot_state->copyJointGroupPositions(joint_model_group, joint_solution);
        return true;
    }
    
    return false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "task_optional_cartesian_planner");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    // Parameters
    std::string group_name;
    pnh.param<std::string>("group_name", group_name, std::string("arm"));
    
    try {
        MotionPlanning planner(nh);
        
        ROS_INFO("========================================");
        ROS_INFO("OPTIONAL TASK: Numerical Cartesian Path Planning");
        ROS_INFO("========================================");
        
        // ===== DEFINE POSE A (Starting pose) =====
        geometry_msgs::Pose pose_A;
        pose_A.position.x = 0.4;
        pose_A.position.y = 0.2;
        pose_A.position.z = 0.3;
        
        // Orientation: pointing down with slight rotation
        tf2::Quaternion q_A;
        q_A.setRPY(M_PI, 0, M_PI/4);  // Roll=180°, Pitch=0°, Yaw=45°
        pose_A.orientation.x = q_A.x();
        pose_A.orientation.y = q_A.y();
        pose_A.orientation.z = q_A.z();
        pose_A.orientation.w = q_A.w();
        
        ROS_INFO("\nPose A (Start):");
        ROS_INFO("  Position: [%.3f, %.3f, %.3f]", 
                 pose_A.position.x, pose_A.position.y, pose_A.position.z);
        ROS_INFO("  Orientation (quaternion): [%.3f, %.3f, %.3f, %.3f]",
                 pose_A.orientation.x, pose_A.orientation.y, 
                 pose_A.orientation.z, pose_A.orientation.w);
        
        // ===== DEFINE POSE B (Goal pose) =====
        geometry_msgs::Pose pose_B;
        pose_B.position.x = 0.4;
        pose_B.position.y = -0.2;
        pose_B.position.z = 0.5;
        
        // Orientation: similar but slightly different
        tf2::Quaternion q_B;
        q_B.setRPY(M_PI, 0, -M_PI/4);  // Roll=180°, Pitch=0°, Yaw=-45°
        pose_B.orientation.x = q_B.x();
        pose_B.orientation.y = q_B.y();
        pose_B.orientation.z = q_B.z();
        pose_B.orientation.w = q_B.w();
        
        ROS_INFO("\nPose B (Goal):");
        ROS_INFO("  Position: [%.3f, %.3f, %.3f]", 
                 pose_B.position.x, pose_B.position.y, pose_B.position.z);
        ROS_INFO("  Orientation (quaternion): [%.3f, %.3f, %.3f, %.3f]",
                 pose_B.orientation.x, pose_B.orientation.y, 
                 pose_B.orientation.z, pose_B.orientation.w);
        
        // ===== STEP 1: Interpolate Cartesian waypoints =====
        ROS_INFO("\n--- STEP 1: Cartesian Interpolation ---");
        double step_size = 0.02;  // 2cm steps
        std::vector<geometry_msgs::Pose> cartesian_waypoints = 
            interpolateCartesianPath(pose_A, pose_B, step_size);
        
        // ===== STEP 2: Solve IK for each waypoint =====
        ROS_INFO("\n--- STEP 2: Inverse Kinematics Solution ---");
        trajectory_msgs::JointTrajectory joint_trajectory;
        
        // Get joint names from move group
        moveit::planning_interface::MoveGroupInterface move_group(group_name);
        const std::vector<std::string>& joint_names = move_group.getJointNames();
        joint_trajectory.joint_names = joint_names;
        
        ROS_INFO("Solving IK for %lu waypoints...", cartesian_waypoints.size());
        
        int ik_failures = 0;
        std::vector<double> previous_joint_solution;
        
        for (size_t i = 0; i < cartesian_waypoints.size(); i++) {
            std::vector<double> joint_solution;
            
            if (solveIK(move_group, cartesian_waypoints[i], joint_solution)) {
                // Check for large jumps (might indicate IK flips)
                if (!previous_joint_solution.empty()) {
                    double max_diff = 0.0;
                    for (size_t j = 0; j < joint_solution.size(); j++) {
                        double diff = std::abs(joint_solution[j] - previous_joint_solution[j]);
                        max_diff = std::max(max_diff, diff);
                    }
                    
                    if (max_diff > M_PI/2) {
                        ROS_WARN("  Large joint jump detected at waypoint %lu: %.3f rad", i, max_diff);
                    }
                }
                
                trajectory_msgs::JointTrajectoryPoint point;
                point.positions = joint_solution;
                point.time_from_start = ros::Duration(i * 0.1);  // 0.1s per waypoint
                joint_trajectory.points.push_back(point);
                
                previous_joint_solution = joint_solution;
            } else {
                ROS_WARN("  IK failed for waypoint %lu", i);
                ik_failures++;
            }
        }
        
        ROS_INFO("IK Success: %lu/%lu waypoints (%.1f%%)", 
                 joint_trajectory.points.size(), 
                 cartesian_waypoints.size(),
                 100.0 * joint_trajectory.points.size() / cartesian_waypoints.size());
        
        if (ik_failures > 0) {
            ROS_WARN("  %d IK failures detected - path may not be fully reachable", ik_failures);
        }
        
        // ===== STEP 3: Optimize trajectory timing =====
        if (!joint_trajectory.points.empty()) {
            ROS_INFO("\n--- STEP 3: Trajectory Optimization ---");
            if (TrajectoryValidatorAndOptimizer::optimizeTrajectory(joint_trajectory, group_name)) {
                ROS_INFO("Trajectory optimized successfully");
            } else {
                ROS_WARN("Trajectory optimization failed, using unoptimized timing");
            }
        }
        
        // ===== STEP 4: Validate trajectory =====
        if (!joint_trajectory.points.empty()) {
            ROS_INFO("\n--- STEP 4: Trajectory Validation ---");
            std::string error_msg;
            bool valid = TrajectoryValidatorAndOptimizer::validateTrajectory(joint_trajectory, group_name, 
                                                   planning_scene::PlanningScenePtr(), &error_msg);
            
            if (valid) {
                ROS_INFO("✓ Trajectory is VALID");
                ROS_INFO("  Waypoints: %lu", joint_trajectory.points.size());
                ROS_INFO("  Duration: %.2f s", joint_trajectory.points.back().time_from_start.toSec());
            } else {
                ROS_WARN("✗ Trajectory validation failed: %s", error_msg.c_str());
            }
            
            // ===== STEP 5: Export trajectory =====
            ROS_INFO("\n--- STEP 5: Export Trajectory ---");
            std::string output_path = "/home/shermin/coding_challenge/src/neura_motion_planning_challenge/utils/cartesian_trajectory.txt";
            std::vector<CartMotionPlanningData> cart_data;
            
            // Store original Cartesian waypoints for visualization
            for (const auto& pose : cartesian_waypoints) {
                CartMotionPlanningData data(pose, 0.0, std_msgs::Header());
                cart_data.push_back(data);
            }
            
            if (planner.exportTrajectoryToFile(output_path, joint_trajectory, cart_data)) {
                ROS_INFO("✓ Trajectory saved to: %s", output_path.c_str());
            }
            
            ROS_INFO("\n========================================");
            ROS_INFO("NUMERICAL CARTESIAN PLANNING COMPLETE");
            ROS_INFO("========================================");
            
            return valid ? 0 : 1;
        } else {
            ROS_ERROR("No valid trajectory points generated - all IK solutions failed");
            return 2;
        }
        
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Exception in task_optional: " << e.what());
        return 2;
    }
}
