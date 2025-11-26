#include <ros/ros.h>
#include <neura_motion_planning_challenge/motion_planning.h>
#include <neura_motion_planning_challenge/PlanMetaData.h>
#include <neura_motion_planning_challenge/PlanCriteria.h>
#include <neura_motion_planning_challenge/PlanComparator.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/move_group_interface/move_group_interface.h>

using namespace neura_motion_planning_challenge;

// Helper function to extract planner name from trajectory filename
std::string extractPlannerNameFromFilepath(const std::string& filepath) {
    size_t start = filepath.find("task_one_") + 9;
    size_t end = filepath.find("_trajectory.yaml");
    return filepath.substr(start, end - start);
}

// Configuration constants
const std::string CONFIG_PATH = "/opt/ros/noetic/share/mira_moveit_config/config/ompl_planning.yaml";
const std::string PACKAGE_PATH = "/home/shermin/coding_challenge/src/neura_motion_planning_challenge";
const std::string PLANNING_GROUP = "arm";

int main(int argc, char** argv)
{
    ros::init(argc, argv, "task_one_node");
    ros::NodeHandle nh;
    
    // Start async spinner for MoveIt to process callbacks
    ros::AsyncSpinner spinner(2);
    spinner.start();
    
    ROS_INFO("\n========================================");
    ROS_INFO("Task 1: Plan to Cartesian Pose with Multiple Planners");
    ROS_INFO("========================================\n");
    
    // Create MotionPlanning instance
    MotionPlanning planner(nh);
    
    // Add collision objects to the scene
    ROS_INFO("\n--- Adding Collision Objects ---");
    
    // Create collision markers array
    visualization_msgs::MarkerArray collision_markers;
    
    // Box 1: position (0.3, 0.3, 1.5), dimensions (0.2 x 0.2 x 0.3)
    visualization_msgs::Marker box1;
    box1.header.frame_id = "base_link";
    box1.header.stamp = ros::Time::now();
    box1.ns = "box1";
    box1.id = 1;
    box1.type = visualization_msgs::Marker::CUBE;
    box1.pose.position.x = 0.3;
    box1.pose.position.y = 0.3;
    box1.pose.position.z = 1.5;
    box1.pose.orientation.w = 1.0;
    box1.scale.x = 0.2;
    box1.scale.y = 0.2;
    box1.scale.z = 0.3;
    collision_markers.markers.push_back(box1);
    
    // Box 2: position (0.3, -0.3, 1.5), dimensions (0.2 x 0.2 x 0.3)
    visualization_msgs::Marker box2;
    box2.header.frame_id = "base_link";
    box2.header.stamp = ros::Time::now();
    box2.ns = "box2";
    box2.id = 2;
    box2.type = visualization_msgs::Marker::CUBE;
    box2.pose.position.x = 0.3;
    box2.pose.position.y = -0.3;
    box2.pose.position.z = 1.5;
    box2.pose.orientation.w = 1.0;
    box2.scale.x = 0.2;
    box2.scale.y = 0.2;
    box2.scale.z = 0.3;
    collision_markers.markers.push_back(box2);
    
    // Add collision objects using MotionPlanning method
    bool collision_added = planner.addCollision(collision_markers);
    if (collision_added) {
        ROS_INFO("✓ Collision objects added to planning scene");
    } else {
        ROS_WARN("✗ Failed to add collision objects");
    }
    
    // Define target Cartesian pose
    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link";
    target_pose.header.stamp = ros::Time::now();
    
    // Set position (x, y, z in meters) - using a known reachable pose
    target_pose.pose.position.x = 0.5;
    target_pose.pose.position.y = 0.0;
    target_pose.pose.position.z = 1.7;
    
    // Set orientation using quaternion - keep similar to current orientation
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, M_PI/2);  // 90 degrees yaw (similar to current pose)
    target_pose.pose.orientation = tf2::toMsg(q);
    
    ROS_INFO("\nTarget pose:");
    ROS_INFO("  Position: x=%.2f, y=%.2f, z=%.2f", 
             target_pose.pose.position.x,
             target_pose.pose.position.y,
             target_pose.pose.position.z);
    ROS_INFO("  Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f\n",
             target_pose.pose.orientation.x,
             target_pose.pose.orientation.y,
             target_pose.pose.orientation.z,
             target_pose.pose.orientation.w);
    
    // Get available planners from OMPL configuration
    ROS_INFO("\n--- Getting Available Planners ---");
    std::string config_path = CONFIG_PATH;
    std::string planning_group = PLANNING_GROUP;
    
    std::vector<std::string> available_planners;
    try {
        available_planners = planner.getAvailablePlanners(config_path, planning_group);
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to get available planners: %s", e.what());
        ROS_WARN("Using default planners instead");
        available_planners = {"RRTConnect", "RRT", "RRTstar", "BKPIECE", "LBKPIECE"};
    }
    
    // Select up to 5 planners to test
    int num_planners = std::min(5, static_cast<int>(available_planners.size()));
    ROS_INFO("Testing %d planners\n", num_planners);
    
    // Storage for trajectories and Plan objects
    std::string package_path = PACKAGE_PATH;
    std::vector<PlanMetadata> plans; // Store all successful plans
    
    // Loop through planners
    for (int i = 0; i < num_planners; i++) {
        std::string planner_id = available_planners[i];
        ROS_INFO("\n--- Testing planner %d/%d: %s ---", i+1, num_planners, planner_id.c_str());
        
        ros::Time start_time = ros::Time::now();
        moveit_msgs::RobotTrajectory trajectory;
        bool success = planner.planPose(target_pose, trajectory, planner_id);
        double planning_time = (ros::Time::now() - start_time).toSec();
        
        if (success) {
            ROS_INFO("✓ Planning succeeded with %s (%.3f seconds)", planner_id.c_str(), planning_time);
            
            // Create filename with planner name
            std::string output_file = package_path + "/trajectories/task_one_" + planner_id + "_trajectory.yaml";
            
            ROS_INFO("Exporting trajectory to: %s", output_file.c_str());
            bool export_success = planner.exportTrajectoryToFile(output_file, trajectory.joint_trajectory, 
                                                                 std::vector<CartMotionPlanningData>());
            
            if (export_success) {
                ROS_INFO("✓ Trajectory successfully exported!");
                
                // Create Plan object and calculate metrics
                PlanMetadata plan(output_file, planning_time, planner_id);
                plan.calculateLength(trajectory.joint_trajectory);
                plan.calculateSumAbsJoints(trajectory.joint_trajectory);
                
                plans.push_back(plan);
            } else {
                ROS_WARN("✗ Failed to export trajectory for %s", planner_id.c_str());
            }
        } else {
            ROS_WARN("✗ Planning failed with %s", planner_id.c_str());
        }
        
        // Small delay between planning attempts
        ros::Duration(0.5).sleep();
    }
    
    // Compare and rank plans
    if (!plans.empty()) {
        ROS_INFO("\n========================================");
        ROS_INFO("Comparing and Ranking Trajectories");
        ROS_INFO("========================================\n");
        
        // Choose criteria for ranking (you can change this)
        PlanCriteria criteria = PlanCriteria::LENGTH;
        ROS_INFO("Ranking criteria: LENGTH\n");
        
        // Sort plans by criteria (refactored to PlanComparator)
        std::vector<PlanMetadata> sorted_plans = PlanComparator::sortPlansByCriteria(plans, criteria);
        
        ROS_INFO("Ranking (best to worst):");
        for (size_t i = 0; i < sorted_plans.size(); i++) {
            ROS_INFO("  %lu. Planner: %s - Length: %.3f, Sum abs joints: %.3f, Time: %.3f s", 
                     i+1,
                     sorted_plans[i].getPlannerId().c_str(),
                     sorted_plans[i].getLength(),
                     sorted_plans[i].getSumAbsJoints(),
                     sorted_plans[i].getPlanTime());
        }
        
        // Get the best plan (refactored to PlanComparator)
        auto best_plan_opt = PlanComparator::getBestPlan(plans, criteria);
        
        if (!best_plan_opt.has_value()) {
            ROS_ERROR("No valid plans found!");
            return 1;
        }
        
        const PlanMetadata& best_plan = best_plan_opt.value();
        
        // Get best planner name from stored planner_id
        std::string best_planner_name = best_plan.getPlannerId();
        
        ROS_INFO("\n*** Best Planner: %s ***", best_planner_name.c_str());
        ROS_INFO("  Trajectory file: %s", best_plan.getTrajectoryFile().c_str());
        ROS_INFO("  Length: %.3f", best_plan.getLength());
        ROS_INFO("  Sum of absolute joints: %.3f", best_plan.getSumAbsJoints());
        ROS_INFO("  Planning time: %.3f seconds\n", best_plan.getPlanTime());
        
        // Load and visualize the best trajectory
        ROS_INFO("Loading and visualizing best trajectory (%s)...", best_planner_name.c_str());
        try {
            TrajectoryData best_data = planner.importTrajectoryFromFile(best_plan.getTrajectoryFile());
            moveit_msgs::RobotTrajectoryPtr best_traj(new moveit_msgs::RobotTrajectory());
            best_traj->joint_trajectory = best_data.getJointTrajectory();
            
            bool viz_success = planner.visualizeJointTrajectory(best_traj);
            if (viz_success) {
                ROS_INFO("✓ Best trajectory visualized in RViz");
            }
            
            // Execute the best trajectory
            ROS_INFO("\nExecuting best trajectory...");
            // Note: Uncomment the line below to actually execute
            // planner.move_group_->execute(*best_traj);
            ROS_INFO("(Execution commented out for safety - uncomment in code to execute)");
            
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to load/visualize best trajectory: %s", e.what());
        }
    } else {
        ROS_ERROR("No successful plans were created! All planning attempts failed.");
        ROS_ERROR("Task 1 cannot proceed without at least one valid plan.");
        ros::shutdown();
        return 1;
    }
    
    ROS_INFO("\n========================================");
    ROS_INFO("Task 1 Testing Complete");
    ROS_INFO("========================================\n");
    
    // Keep node alive for visualization
    ros::Duration(5.0).sleep();
    
    ros::shutdown();
    return 0;
}
