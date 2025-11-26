// Optional Task: Cartesian path planning using numerical approach
// Tests Step 1 of CartMotionPlanner: Waypoint sampling between two Cartesian poses

#include <ros/ros.h>
#include <neura_motion_planning_challenge/Planning/CartMotionPlanner.h>
#include <neura_motion_planning_challenge/Planning/motion_planning.h>
#include <neura_motion_planning_challenge/DataStructure/CartMotionPlanningData.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Header.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <iomanip>
#include <sstream>

using namespace neura_motion_planning_challenge;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "task_optional_cartesian_planner");
    ros::NodeHandle nh;
    
    ROS_INFO("\n========================================");
    ROS_INFO("Task Optional: Test CartMotionPlanner Step 1 (Sampling)");
    ROS_INFO("========================================\n");
    
    // Create header for waypoints
    std_msgs::Header header;
    header.frame_id = "base_link";
    header.stamp = ros::Time::now();
    
    // Define Waypoint A using Box 1 position: (0.3, 0.3, 1.5)
    ROS_INFO("\n--- Creating Waypoint A (Box 1 position) ---");
    geometry_msgs::Pose pose_A;
    pose_A.position.x = 0.3;
    pose_A.position.y = 0.3;
    pose_A.position.z = 1.5;
    
    // Set identity orientation (no rotation)
    tf2::Quaternion quat_A;
    quat_A.setRPY(0.0, 0.0, 0.0);
    pose_A.orientation = tf2::toMsg(quat_A);
    
    CartMotionPlanningData waypoint_A(pose_A, 0.0, header);
    ROS_INFO("Waypoint A: position (%.3f, %.3f, %.3f)", 
             pose_A.position.x, pose_A.position.y, pose_A.position.z);
    ROS_INFO("           orientation (%.3f, %.3f, %.3f, %.3f)",
             pose_A.orientation.x, pose_A.orientation.y, pose_A.orientation.z, pose_A.orientation.w);
    
    // Define Waypoint B using Box 2 position: (0.3, -0.3, 1.5)
    ROS_INFO("\n--- Creating Waypoint B (Box 2 position) ---");
    geometry_msgs::Pose pose_B;
    pose_B.position.x = 0.3;
    pose_B.position.y = -0.3;
    pose_B.position.z = 1.5;
    
    // Set 90 degree yaw rotation
    tf2::Quaternion quat_B;
    quat_B.setRPY(0.0, 0.0, M_PI/2);
    pose_B.orientation = tf2::toMsg(quat_B);
    
    CartMotionPlanningData waypoint_B(pose_B, 1.0, header);
    ROS_INFO("Waypoint B: position (%.3f, %.3f, %.3f)", 
             pose_B.position.x, pose_B.position.y, pose_B.position.z);
    ROS_INFO("           orientation (%.3f, %.3f, %.3f, %.3f)",
             pose_B.orientation.x, pose_B.orientation.y, pose_B.orientation.z, pose_B.orientation.w);
    
    // Test Step 1: Call planNumericalCartesianPath with only sampling (no collision checking)
    ROS_INFO("\n--- Testing Step 1: Sampling with default UniformSampler ---");
    int num_waypoints = 5;
    ROS_INFO("Requesting %d waypoints between A and B...", num_waypoints);
    
    moveit_msgs::RobotTrajectory trajectory = CartMotionPlanner::planNumericalCartesianPath(
        waypoint_A,
        waypoint_B,
        num_waypoints,
        false,  // No collision checking (just test sampling)
        nullptr // Use default UniformSampler
    );
    
    ROS_INFO("\n--- Step 1 Results ---");
    if (trajectory.joint_trajectory.points.empty()) {
        ROS_ERROR("FAILED: No waypoints generated!");
        return 1;
    }
    
    size_t num_points = trajectory.joint_trajectory.points.size();
    ROS_INFO("SUCCESS: Generated %zu waypoints (requested %d)", num_points, num_waypoints);
    
    // Export the first trajectory
    ROS_INFO("\n--- Exporting first trajectory ---");
    MotionPlanning planner(nh);
    std::string export_path_1 = "/home/shermin/coding_challenge/src/neura_motion_planning_challenge/trajectories/task_optional_trajectory_1.yaml";
    if (planner.exportTrajectoryToFile(export_path_1, trajectory.joint_trajectory, std::vector<CartMotionPlanningData>())) {
        ROS_INFO("✓ First trajectory exported to: %s", export_path_1.c_str());
    } else {
        ROS_WARN("✗ Failed to export first trajectory");
    }
    
    // Visualize the first trajectory in RViz
    ROS_INFO("\n--- Visualizing first trajectory in RViz ---");
    moveit_msgs::RobotTrajectoryPtr viz_traj_1(new moveit_msgs::RobotTrajectory());
    viz_traj_1->joint_trajectory = trajectory.joint_trajectory;
    if (planner.visualizeJointTrajectory(viz_traj_1)) {
        ROS_INFO("✓ First trajectory visualized in RViz");
    } else {
        ROS_WARN("✗ Failed to visualize first trajectory");
    }
    
    // Keep visualization on screen for a few seconds before moving on
    ros::Duration(2.0).sleep();
    
    // Display joint trajectory details
    ROS_INFO("\nJoint Trajectory Details:");
    ROS_INFO("  Frame ID: %s", trajectory.joint_trajectory.header.frame_id.c_str());
    ROS_INFO("  Number of joints: %zu", trajectory.joint_trajectory.joint_names.size());
    ROS_INFO("  Number of waypoints: %zu", num_points);
    
    // Display waypoint details
    ROS_INFO("\nJoint Trajectory Waypoints:");
    for (size_t i = 0; i < num_points; ++i) {
        const auto& point = trajectory.joint_trajectory.points[i];
        ROS_INFO("  Waypoint %zu:", i);
        ROS_INFO("    Time from start: %.3f s", point.time_from_start.toSec());
        ROS_INFO("    Number of joint positions: %zu", point.positions.size());
        
        // Display joint positions
        if (!point.positions.empty()) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(4);
            for (size_t j = 0; j < std::min(size_t(6), point.positions.size()); ++j) {
                oss << point.positions[j];
                if (j < std::min(size_t(5), point.positions.size() - 1)) oss << ", ";
            }
            if (point.positions.size() > 6) oss << ", ...";
            ROS_INFO("    Joint positions: [%s]", oss.str().c_str());
        }
    }
    
    // Also display the Cartesian interpolation path
    ROS_INFO("\nCartesian Path Interpolation (from A to B):");
    ROS_INFO("  Start (Waypoint A): position (%.3f, %.3f, %.3f)", 
             waypoint_A.getPose().position.x, waypoint_A.getPose().position.y, waypoint_A.getPose().position.z);
    ROS_INFO("  End   (Waypoint B): position (%.3f, %.3f, %.3f)", 
             waypoint_B.getPose().position.x, waypoint_B.getPose().position.y, waypoint_B.getPose().position.z);
    ROS_INFO("  Interpolated %zu waypoints along this Cartesian path", num_points);
    
    // Test with custom sampler (test polymorphism)
    ROS_INFO("\n--- Testing Step 1: Sampling with explicit nullptr (should use default) ---");
    moveit_msgs::RobotTrajectory trajectory2 = CartMotionPlanner::planNumericalCartesianPath(
        waypoint_A,
        waypoint_B,
        8,  // Different number of waypoints
        false,
        nullptr // Explicitly pass nullptr
    );
    
    if (trajectory2.joint_trajectory.points.size() == 8) {
        ROS_INFO("SUCCESS: Generated %zu waypoints as expected", trajectory2.joint_trajectory.points.size());
    } else {
        ROS_ERROR("FAILED: Expected 8 waypoints, got %zu", trajectory2.joint_trajectory.points.size());
        return 1;
    }
    
    // Export the second trajectory
    ROS_INFO("\n--- Exporting second trajectory ---");
    std::string export_path_2 = "/home/shermin/coding_challenge/src/neura_motion_planning_challenge/trajectories/task_optional_trajectory_2.yaml";
    if (planner.exportTrajectoryToFile(export_path_2, trajectory2.joint_trajectory, std::vector<CartMotionPlanningData>())) {
        ROS_INFO("✓ Second trajectory exported to: %s", export_path_2.c_str());
    } else {
        ROS_WARN("✗ Failed to export second trajectory");
    }
    
    // Visualize the second trajectory in RViz
    ROS_INFO("\n--- Visualizing second trajectory in RViz ---");
    moveit_msgs::RobotTrajectoryPtr viz_traj_2(new moveit_msgs::RobotTrajectory());
    viz_traj_2->joint_trajectory = trajectory2.joint_trajectory;
    if (planner.visualizeJointTrajectory(viz_traj_2)) {
        ROS_INFO("✓ Second trajectory visualized in RViz");
    } else {
        ROS_WARN("✗ Failed to visualize second trajectory");
    }
    
    // Keep visualization on screen for a few seconds before completion
    ros::Duration(2.0).sleep();
    
    ROS_INFO("\n========================================");
    ROS_INFO("Step 1 Testing COMPLETE - Sampling works correctly!");
    ROS_INFO("========================================\n");
    
    return 0;
}
