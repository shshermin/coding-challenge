#include <neura_motion_planning_challenge/Trajectory/TrajectoryVisualizer.h>
#include <neura_motion_planning_challenge/DataStructure/CartMotionPlanningData.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <visualization_msgs/MarkerArray.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

using namespace neura_motion_planning_challenge;

// Helper method: Publish trajectory to RViz for visualization
void TrajectoryVisualizer::publishTrajectoryToRViz(const moveit::planning_interface::MoveGroupInterface::Plan& plan,
                                             ros::Publisher& display_publisher)
{
    moveit_msgs::DisplayTrajectory display_trajectory;
    display_trajectory.trajectory_start = plan.start_state_;
    display_trajectory.trajectory.push_back(plan.trajectory_);
    display_trajectory.model_id = "mira";
    
    // Publish multiple times to ensure RViz receives it
    for (int i = 0; i < 5; i++) {
        display_publisher.publish(display_trajectory);
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }
    
    ROS_INFO("Trajectory published to RViz (%d subscribers)", display_publisher.getNumSubscribers());
}

// Helper method: Log detailed trajectory information
void TrajectoryVisualizer::logTrajectoryDetails(const trajectory_msgs::JointTrajectory& trajectory)
{
    ROS_INFO("\n=== TRAJECTORY DETAILS ===");
    ROS_INFO("Waypoints: %lu | Duration: %.2f s", 
             trajectory.points.size(), 
             trajectory.points.back().time_from_start.toSec());
    
    // Log first and last waypoint positions
    if (!trajectory.points.empty()) {
        const auto& first = trajectory.points.front();
        const auto& last = trajectory.points.back();
        
        ROS_INFO("Start positions (rad): [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                 first.positions[0], first.positions[1], first.positions[2], 
                 first.positions[3], first.positions[4], first.positions[5], first.positions[6]);
        
        ROS_INFO("End positions (rad):   [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                 last.positions[0], last.positions[1], last.positions[2], 
                 last.positions[3], last.positions[4], last.positions[5], last.positions[6]);
    }
    ROS_INFO("=== END DETAILS ===\n");
}

// Helper method: Visualize and log trajectory with success message
void TrajectoryVisualizer::logAndVisualizeTrajectory(const moveit::planning_interface::MoveGroupInterface::Plan& plan,
                                                      ros::Publisher& display_publisher,
                                                      const std::string& method_name)
{
    publishTrajectoryToRViz(plan, display_publisher);
    logTrajectoryDetails(plan.trajectory_.joint_trajectory);
    ROS_INFO("%s succeeded with %lu waypoints", method_name.c_str(), plan.trajectory_.joint_trajectory.points.size());
}

// Visualize joint trajectory in RViz
bool TrajectoryVisualizer::visualizeJointTrajectory(const moveit_msgs::RobotTrajectoryPtr& joint_trajectory,
                                                     ros::Publisher& display_publisher,
                                                     const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group_ptr)
{
    if (!joint_trajectory) {
        ROS_ERROR("Cannot visualize null trajectory");
        return false;
    }
    
    try {
        moveit_msgs::DisplayTrajectory display_trajectory;
        
        // Get current robot state as the starting state
        if (move_group_ptr) {
            robot_state::RobotStatePtr current_state = move_group_ptr->getCurrentState(3.0);
            if (current_state) {
                moveit::core::robotStateToRobotStateMsg(*current_state, display_trajectory.trajectory_start);
            }
        }
        
        display_trajectory.trajectory.push_back(*joint_trajectory);
        display_trajectory.model_id = "mira";
        
        // Publish multiple times to ensure RViz receives it
        for (int i = 0; i < 5; i++) {
            display_publisher.publish(display_trajectory);
            ros::Duration(0.1).sleep();
            ros::spinOnce();
        }
        
        ROS_INFO("Joint trajectory visualized in RViz (%d subscribers)", display_publisher.getNumSubscribers());
        return true;
        
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in visualizeJointTrajectory: %s", e.what());
        return false;
    }
}

// Visualize Cartesian waypoints as sphere markers
bool TrajectoryVisualizer::visualizeCartTrajectory(const std::vector<CartMotionPlanningData>& cart_pose_array)
{
    if (cart_pose_array.empty()) {
        ROS_WARN("Cannot visualize empty waypoint array");
        return false;
    }
    
    try {
        // Create a marker array to hold all waypoint markers
        visualization_msgs::MarkerArray marker_array;
        
        for (size_t i = 0; i < cart_pose_array.size(); ++i) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "base_link";
            marker.header.stamp = ros::Time::now();
            marker.ns = "cartesian_waypoints";
            marker.id = i;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            
            // Set pose from waypoint
            marker.pose = cart_pose_array[i].getPose();
            
            // Set uniform scale for all waypoints (0.05m diameter spheres)
            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            
            // Color coding: Green for start, Red for end, Yellow for intermediate
            if (i == 0) {
                // Green for start waypoint
                marker.color.r = 0.0f;
                marker.color.g = 1.0f;
                marker.color.b = 0.0f;
                marker.color.a = 0.8f;
            } else if (i == cart_pose_array.size() - 1) {
                // Red for end waypoint
                marker.color.r = 1.0f;
                marker.color.g = 0.0f;
                marker.color.b = 0.0f;
                marker.color.a = 0.8f;
            } else {
                // Yellow for intermediate waypoints
                marker.color.r = 1.0f;
                marker.color.g = 1.0f;
                marker.color.b = 0.0f;
                marker.color.a = 0.6f;
            }
            
            // Set lifetime (0 means persist indefinitely until next update)
            marker.lifetime = ros::Duration(0);
            
            marker_array.markers.push_back(marker);
        }
        
        // Create publisher for waypoint markers if not already done
        static ros::Publisher waypoint_publisher;
        static bool publisher_initialized = false;
        
        if (!publisher_initialized) {
            // Need a node handle for publisher creation - using a local one
            ros::NodeHandle nh;
            waypoint_publisher = nh.advertise<visualization_msgs::MarkerArray>("cartesian_waypoints", 1, true);
            publisher_initialized = true;
        }
        
        // Publish the markers multiple times to ensure RViz receives them
        for (int i = 0; i < 5; i++) {
            waypoint_publisher.publish(marker_array);
            ros::Duration(0.1).sleep();
            ros::spinOnce();
        }
        
        ROS_INFO("Cartesian trajectory visualized in RViz - %lu waypoints with color coding:", cart_pose_array.size());
        ROS_INFO("  Green: Start waypoint");
        ROS_INFO("  Yellow: Intermediate waypoints");
        ROS_INFO("  Red: End waypoint");
        
        return true;
        
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in visualizeCartTrajectory: %s", e.what());
        return false;
    }
}
