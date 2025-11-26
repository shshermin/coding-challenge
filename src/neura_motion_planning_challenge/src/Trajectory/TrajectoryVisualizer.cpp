#include <neura_motion_planning_challenge/Trajectory/TrajectoryVisualizer.h>
#include <moveit_msgs/DisplayTrajectory.h>

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
