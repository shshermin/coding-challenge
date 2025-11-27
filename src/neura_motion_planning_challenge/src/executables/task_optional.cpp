// Optional Task: Cartesian path planning using numerical approach
// Plans a Cartesian path between two waypoints and visualizes the result

#include <ros/ros.h>
#include <neura_motion_planning_challenge/Planning/motion_planning.h>
#include <neura_motion_planning_challenge/Trajectory/TrajectoryVisualizer.h>
#include <neura_motion_planning_challenge/Trajectory/TrajectoryIO.h>
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
    
    // Create MotionPlanning instance
    MotionPlanning motion_planner(nh);
    
    ROS_INFO("\n========================================");
    ROS_INFO("Task Optional: Cartesian Path Planning");
    ROS_INFO("========================================\n");
    
    // Create header for waypoints
    std_msgs::Header header;
    header.frame_id = "base_link";
    header.stamp = ros::Time::now();
    
    // Define starting Cartesian waypoint
    ROS_INFO("\n--- Creating Start Waypoint ---");
    geometry_msgs::Pose pose_start;
    pose_start.position.x = 0.3;
    pose_start.position.y = 0.3;
    pose_start.position.z = 1.5;
    
    // Set identity orientation
    tf2::Quaternion quat_start;
    quat_start.setRPY(0.0, 0.0, 0.0);
    pose_start.orientation = tf2::toMsg(quat_start);
    
    CartMotionPlanningData waypoint_start(pose_start, 0.0, header);
    ROS_INFO("Start Waypoint: position (%.3f, %.3f, %.3f)", 
             pose_start.position.x, pose_start.position.y, pose_start.position.z);
    
    // Define end Cartesian waypoint
    ROS_INFO("\n--- Creating End Waypoint ---");
    geometry_msgs::Pose pose_end;
    pose_end.position.x = 0.3;
    pose_end.position.y = -0.3;
    pose_end.position.z = 1.5;
    
    // Set 90 degree yaw rotation
    tf2::Quaternion quat_end;
    quat_end.setRPY(0.0, 0.0, M_PI/2);
    pose_end.orientation = tf2::toMsg(quat_end);
    
    CartMotionPlanningData waypoint_end(pose_end, 1.0, header);
    ROS_INFO("End Waypoint: position (%.3f, %.3f, %.3f)", 
             pose_end.position.x, pose_end.position.y, pose_end.position.z);
    
    // Plan Cartesian path between waypoints
    ROS_INFO("\n--- Planning Cartesian Path ---");
    int num_waypoints = 10;
    ROS_INFO("Requesting %d intermediate waypoints...", num_waypoints);
    
    // Store waypoints for visualization
    std::vector<CartMotionPlanningData> waypoint_start_vec = {waypoint_start};
    std::vector<CartMotionPlanningData> waypoint_end_vec = {waypoint_end};
    
    moveit_msgs::RobotTrajectory trajectory;
    bool success = motion_planner.planNumericalCartesianPath(
        waypoint_start,
        waypoint_end,
        trajectory,
        num_waypoints,
        false,  // No collision checking
        nullptr // Use default UniformSampler
    );
    
    // Check if planning was successful
    if (!success || trajectory.joint_trajectory.points.empty()) {
        ROS_ERROR("FAILED: No waypoints generated!");
        return 1;
    }
    
    size_t num_points = trajectory.joint_trajectory.points.size();
    ROS_INFO("SUCCESS: Generated %zu waypoints", num_points);
    
    // Display trajectory details
    ROS_INFO("\n--- Trajectory Details ---");
    ROS_INFO("  Frame ID: %s", trajectory.joint_trajectory.header.frame_id.c_str());
    ROS_INFO("  Number of joints: %zu", trajectory.joint_trajectory.joint_names.size());
    ROS_INFO("  Number of waypoints: %zu", num_points);
    
    // Visualize Cartesian waypoints in RViz
    ROS_INFO("\n--- Visualizing Cartesian Waypoints in RViz ---");
    std::vector<CartMotionPlanningData> all_waypoints;
    all_waypoints.push_back(waypoint_start);
    all_waypoints.push_back(waypoint_end);
    
    if (TrajectoryVisualizer::visualizeCartTrajectory(all_waypoints)) {
        ROS_INFO("✓ Cartesian waypoints visualized in RViz (Green=Start, Red=End)");
    } else {
        ROS_WARN("✗ Failed to visualize Cartesian waypoints");
    }
    
    // Visualize trajectory in RViz
    ROS_INFO("\n--- Visualizing Joint Trajectory in RViz ---");
    moveit_msgs::RobotTrajectoryPtr viz_trajectory(new moveit_msgs::RobotTrajectory());
    viz_trajectory->joint_trajectory = trajectory.joint_trajectory;
    if (TrajectoryVisualizer::visualizeJointTrajectory(
            viz_trajectory,
            motion_planner.getDisplayPublisher(),
            motion_planner.getMoveGroup())) {
        ROS_INFO("✓ Trajectory visualized in RViz");
    } else {
        ROS_WARN("✗ Failed to visualize trajectory");
    }
    
    // Keep visualization on screen
    ROS_INFO("\nVisualization active - Press Ctrl+C to exit");
    ros::spin();
    
    return 0;
}
