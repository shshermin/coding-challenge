#include <neura_motion_planning_challenge/UniformSampler.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/ros.h>

namespace neura_motion_planning_challenge
{

/**
 * @brief Generates uniformly spaced waypoints between two Cartesian poses.
 *
 * This method divides the path from start to end into num_waypoints evenly distributed poses.
 * For each waypoint at parameter t (0 to 1):
 *   - Position: Linear interpolation (straight line in 3D space)
 *   - Orientation: SLERP (smooth spherical interpolation of quaternions)
 *
 * This ensures a smooth Cartesian path that can be used for IK solving and motion planning.
 */
std::vector<CartMotionPlanningData> UniformSampler::generateWaypoints(
    const CartMotionPlanningData &start,
    const CartMotionPlanningData &end,
    int num_waypoints)
{
    // Validate input
    if (num_waypoints < 2) {
        ROS_ERROR("UniformSampler: num_waypoints must be at least 2");
        throw std::runtime_error("num_waypoints must be at least 2");
    }

    std::vector<CartMotionPlanningData> waypoints;
    
    // Get start and end poses
    const geometry_msgs::Pose &start_pose = start.getPose();
    const geometry_msgs::Pose &end_pose = end.getPose();
    
    for (int i = 0; i < num_waypoints; ++i) {
        double t = (double)i / (num_waypoints - 1);  // t ranges from 0.0 to 1.0
        
        // ===== LINEAR INTERPOLATION FOR POSITION =====
        geometry_msgs::Pose interpolated_pose;
        
        interpolated_pose.position.x = start_pose.position.x + 
                                        t * (end_pose.position.x - start_pose.position.x);
        interpolated_pose.position.y = start_pose.position.y + 
                                        t * (end_pose.position.y - start_pose.position.y);
        interpolated_pose.position.z = start_pose.position.z + 
                                        t * (end_pose.position.z - start_pose.position.z);
        
        // ===== SLERP (SPHERICAL LINEAR INTERPOLATION) FOR ORIENTATION =====
        // Convert geometry_msgs quaternions to tf2 quaternions
        tf2::Quaternion start_quat(start_pose.orientation.x, 
                                   start_pose.orientation.y,
                                   start_pose.orientation.z, 
                                   start_pose.orientation.w);
        
        tf2::Quaternion end_quat(end_pose.orientation.x, 
                                 end_pose.orientation.y,
                                 end_pose.orientation.z, 
                                 end_pose.orientation.w);
        
        // Perform SLERP at parameter t
        tf2::Quaternion interpolated_quat = start_quat.slerp(end_quat, t);
        
        // Convert back to geometry_msgs quaternion
        interpolated_pose.orientation.x = interpolated_quat.x();
        interpolated_pose.orientation.y = interpolated_quat.y();
        interpolated_pose.orientation.z = interpolated_quat.z();
        interpolated_pose.orientation.w = interpolated_quat.w();
        
        // ===== CREATE CARTESIAN WAYPOINT DATA =====
        // Create header (can be customized as needed)
        std_msgs::Header header;
        header.frame_id = start.getHeader().frame_id;  // Use start waypoint's frame
        header.stamp = ros::Time::now();
        
        // Create CartMotionPlanningData with interpolated pose and time parameter
        CartMotionPlanningData waypoint(interpolated_pose, t, header);
        waypoints.push_back(waypoint);
        
        ROS_DEBUG("UniformSampler: Generated waypoint %d, t=%.3f, pos=(%.3f, %.3f, %.3f)", 
                  i, t, interpolated_pose.position.x, interpolated_pose.position.y, 
                  interpolated_pose.position.z);
    }
    
    ROS_INFO("UniformSampler: Generated %ld waypoints", waypoints.size());
    return waypoints;
}

} // namespace neura_motion_planning_challenge