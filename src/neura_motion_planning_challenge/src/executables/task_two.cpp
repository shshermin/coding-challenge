// Task 2 executable: load a trajectory, validate it with MoveIt, and report.
#include <ros/ros.h>
#include <neura_motion_planning_challenge/Planning/motion_planning.h>
#include <neura_motion_planning_challenge/Trajectory/TrajectoryIO.h>
#include <neura_motion_planning_challenge/DataStructure/TrajectoryData.h>
#include <neura_motion_planning_challenge/DataStructure/PlanMetaData.h>
#include <neura_motion_planning_challenge/Utility/ValidatorAndOptimizer.h>
#include <string>

using namespace neura_motion_planning_challenge;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "task_two_validate_trajectory");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	// Parameters
	std::string group_name;
	pnh.param<std::string>("group_name", group_name, std::string("arm"));

	// Default to the known path in this workspace; allow override via param
	std::string traj_path_default = "/home/shermin/coding_challenge/src/neura_motion_planning_challenge/utils/trajectory.json";
	std::string traj_path;
	pnh.param<std::string>("traj_path", traj_path, traj_path_default);
	
	// Output directory for results (trajectory and plots)
	std::string output_dir_default = "/home/shermin/coding_challenge/src/neura_motion_planning_challenge/utils";
	std::string output_dir;
	pnh.param<std::string>("output_dir", output_dir, output_dir_default);

	try {
		MotionPlanning planner(nh);
		
		// Import original trajectory
		ROS_INFO("========================================");
		ROS_INFO("TASK 2: Trajectory Analysis & Optimization");
		ROS_INFO("========================================");
		
		TrajectoryData data = TrajectoryIO::importTrajectoryFromFile(traj_path);
		trajectory_msgs::JointTrajectory original_traj = data.getJointTrajectory();
		
		// Validate original trajectory
		ROS_INFO("\n--- ORIGINAL TRAJECTORY VALIDATION ---");
		std::string error_msg;
		bool original_valid = TrajectoryValidatorAndOptimizer::validateTrajectory(original_traj, group_name, 
		                                                planning_scene::PlanningScenePtr(), &error_msg);
		
		if (!original_valid) {
			ROS_WARN("Original trajectory has issues: %s", error_msg.c_str());
			ROS_INFO("\n--- ATTEMPTING TRAJECTORY OPTIMIZATION ---");
			
			// Create a copy for optimization
			trajectory_msgs::JointTrajectory optimized_traj = original_traj;
			
			// Optimize trajectory using both TOTG and Parabolic methods
			if (TrajectoryValidatorAndOptimizer::optimizeTrajectory(optimized_traj, group_name, true, true)) {
				ROS_INFO("\n--- OPTIMIZED TRAJECTORY VALIDATION ---");
				std::string opt_error_msg;
				bool optimized_valid = TrajectoryValidatorAndOptimizer::validateTrajectory(optimized_traj, group_name,
					                                         planning_scene::PlanningScenePtr(), &opt_error_msg);
				
			// Compare trajectories
			ROS_INFO("\n========================================");
			ROS_INFO("TRAJECTORY COMPARISON");
			ROS_INFO("========================================");
			
			// Extract durations once for reuse
			double orig_duration = original_traj.points.back().time_from_start.toSec();
			double opt_duration = optimized_traj.points.back().time_from_start.toSec();
			
			ROS_INFO("Original trajectory:");
			ROS_INFO("  - Waypoints: %lu", original_traj.points.size());
			ROS_INFO("  - Duration: %.2f s", orig_duration);
			ROS_INFO("  - Valid: %s", original_valid ? "YES" : "NO");
			
			ROS_INFO("\nOptimized trajectory:");
			ROS_INFO("  - Waypoints: %lu", optimized_traj.points.size());
			ROS_INFO("  - Duration: %.2f s", opt_duration);
			ROS_INFO("  - Valid: %s", optimized_valid ? "YES" : "NO");
			
			ROS_INFO("\nImprovement:");
			double duration_change = opt_duration - orig_duration;
			ROS_INFO("  - Duration change: %+.2f s (%.1f%%)", 
			         duration_change,
			         (duration_change / orig_duration) * 100.0);
			
			// Export optimized trajectory if valid
			if (optimized_valid) {
				std::string output_path = output_dir + "/trajectory_optimized.txt";
				std::vector<CartMotionPlanningData> empty_cart;
				if (TrajectoryIO::exportTrajectoryToFile(output_path, optimized_traj, empty_cart)) {
					ROS_INFO("\n✓ Optimized trajectory saved to: %s", output_path.c_str());
				}
			}
			
			// Generate comparison plot
			ROS_INFO("\n--- GENERATING TRAJECTORY COMPARISON PLOT ---");
			
			// Create PlanMetadata objects and calculate all metrics
			PlanMetadata original_plan;
			original_plan.calculateAllMetrics(original_traj, orig_duration);
			
			PlanMetadata optimized_plan;
			optimized_plan.calculateAllMetrics(optimized_traj, opt_duration);			// Generate comparison plot
			if (TrajectoryValidatorAndOptimizer::generateTrajectoryComparisonPlot(
					original_plan, optimized_plan, output_dir)) {
				ROS_INFO("✓ Comparison plot saved successfully");
			} else {
				ROS_WARN("Failed to generate comparison plot (matplotlib may not be installed)");
			}
			
			return optimized_valid ? 0 : 1;
			} else {
				ROS_ERROR("Trajectory optimization failed");
				return 2;
			}
		} else {
			ROS_INFO("\n✓ Original trajectory is already valid!");
			return 0;
		}
		
	}
	catch (const std::exception& e) {
		ROS_ERROR_STREAM("Exception in task_two: " << e.what());
		return 2;
	}
}