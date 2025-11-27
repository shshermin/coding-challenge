#include <neura_motion_planning_challenge/Trajectory/TrajectoryIO.h>
#include <neura_motion_planning_challenge/DataStructure/TrajectoryData.h>
#include <neura_motion_planning_challenge/Trajectory/TrajectoryParser.h>
#include <fstream>

using namespace neura_motion_planning_challenge;

// Helper: Write vector as comma-separated array
void TrajectoryIO::writeVectorToStream(std::ostream& stream, const std::vector<double>& vec)
{
    stream << "[";
    for (size_t i = 0; i < vec.size(); ++i) {
        stream << vec[i];
        if (i < vec.size() - 1) stream << ", ";
    }
    stream << "]";
}

// Helper: Write joint trajectory section
void TrajectoryIO::writeJointTrajectory(std::ostream& file, const trajectory_msgs::JointTrajectory& joint_trajectory)
{
    file << "joint_trajectory:\n"
         << "  joint_names: ";
    
    // Write joint names
    file << "[";
    for (size_t i = 0; i < joint_trajectory.joint_names.size(); ++i) {
        file << joint_trajectory.joint_names[i];
        if (i < joint_trajectory.joint_names.size() - 1) file << ", ";
    }
    file << "]\n"
         << "  num_waypoints: " << joint_trajectory.points.size() << "\n"
         << "  waypoints:\n";
    
    // Write each waypoint
    for (size_t i = 0; i < joint_trajectory.points.size(); ++i) {
        const auto& point = joint_trajectory.points[i];
        file << "    - index: " << i << "\n"
             << "      time_from_start: " << point.time_from_start.toSec() << "\n"
             << "      positions: ";
        writeVectorToStream(file, point.positions);
        file << "\n";
        
        if (!point.velocities.empty()) {
            file << "      velocities: ";
            writeVectorToStream(file, point.velocities);
            file << "\n";
        }
        
        if (!point.accelerations.empty()) {
            file << "      accelerations: ";
            writeVectorToStream(file, point.accelerations);
            file << "\n";
        }
    }
}

// Helper: Write Cartesian poses section
void TrajectoryIO::writeCartesianPoses(std::ostream& file, const std::vector<CartMotionPlanningData>& cart_pose_array)
{
    file << "\ncartesian_poses:\n"
         << "  num_poses: " << cart_pose_array.size() << "\n"
         << "  poses:\n";
    
    for (size_t i = 0; i < cart_pose_array.size(); ++i) {
        const auto& pose = cart_pose_array[i].getPose();
        const auto& header = cart_pose_array[i].getHeader();
        const auto& time_from_start = cart_pose_array[i].getTimeFromStart();
        
        file << "    - index: " << i << "\n"
             << "      time_from_start: " << time_from_start.toSec() << "\n"
             << "      frame_id: " << header.frame_id << "\n"
             << "      position:\n"
             << "        x: " << pose.position.x << "\n"
             << "        y: " << pose.position.y << "\n"
             << "        z: " << pose.position.z << "\n"
             << "      orientation:\n"
             << "        x: " << pose.orientation.x << "\n"
             << "        y: " << pose.orientation.y << "\n"
             << "        z: " << pose.orientation.z << "\n"
             << "        w: " << pose.orientation.w << "\n";
    }
}

/**
 * @brief Exports a trajectory (joint and Cartesian) to a file in YAML format.
 * @param filepath The destination file path where the trajectory will be saved.
 * @param joint_trajectory The joint trajectory data to export.
 * @param cart_pose_array Vector of Cartesian waypoints to export.
 * @return true if export was successful, false otherwise.
 */
bool TrajectoryIO::exportTrajectoryToFile(const std::string& filepath, const trajectory_msgs::JointTrajectory& joint_trajectory, const std::vector<CartMotionPlanningData>& cart_pose_array)
{
    try {
        std::ofstream file(filepath);
        if (!file.is_open()) {
            ROS_ERROR("Failed to open file for writing: %s", filepath.c_str());
            return false;
        }
        
        // Write header
        file << "# Robot Trajectory Export\n"
             << "# Generated at: " << ros::Time::now() << "\n"
             << "# File format: YAML-like structure\n"
             << "---\n\n";
        
        // Export joint trajectory and Cartesian poses using helper methods
        neura_motion_planning_challenge::TrajectoryIO::writeJointTrajectory(file, joint_trajectory);
        neura_motion_planning_challenge::TrajectoryIO::writeCartesianPoses(file, cart_pose_array);
        
        file.close();
        ROS_INFO("Successfully exported trajectory to: %s", filepath.c_str());
        ROS_INFO("  Joint waypoints: %lu", joint_trajectory.points.size());
        ROS_INFO("  Cartesian poses: %lu", cart_pose_array.size());
        return true;
        
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in exportTrajectoryToFile: %s", e.what());
        return false;
    }
}



/**
 * @brief Imports a trajectory from a file in JSON or YAML format.
 * @param filepath The source file path containing the trajectory data.
 * @return TrajectoryData object populated with the imported trajectory.
 * @throws std::runtime_error If the file cannot be opened or parsing fails.
 */
TrajectoryData TrajectoryIO::importTrajectoryFromFile(const std::string& filepath)
{
    TrajectoryData trajectory_data;
    
    try {
        trajectory_msgs::JointTrajectory& joint_traj = trajectory_data.getJointTrajectory();
        
        // Try JSON format first (check file extension)
        if (filepath.length() >= 5 && filepath.substr(filepath.length() - 5) == ".json") {
            ROS_INFO("Detected JSON format, parsing...");
            if (neura_motion_planning_challenge::TrajectoryParser::parseJsonTrajectory(filepath, joint_traj)) {
                ROS_INFO("Successfully imported trajectory from: %s", filepath.c_str());
                ROS_INFO("  Joint waypoints loaded: %lu", joint_traj.points.size());
                return trajectory_data;
            } else {
                ROS_WARN("JSON parsing failed, trying YAML format...");
            }
        }
        
        // Fall back to YAML-like format
        std::ifstream file(filepath);
        if (!file.is_open()) {
            ROS_ERROR("Failed to open file for reading: %s", filepath.c_str());
            throw std::runtime_error("Cannot open trajectory file");
        }
        
        std::string line;
        bool in_joint_section = false;
        bool in_cart_section = false;
        
        while (std::getline(file, line)) {
            // Skip comments and empty lines
            if (line.empty() || line[0] == '#' || line.find("---") == 0) {
                continue;
            }
            
            // Detect sections
            if (line.find("joint_trajectory:") != std::string::npos) {
                in_joint_section = true;
                in_cart_section = false;
                continue;
            }
            if (line.find("cartesian_poses:") != std::string::npos) {
                in_joint_section = false;
                in_cart_section = true;
                continue;
            }
            
            // Parse joint trajectory section
            if (in_joint_section) {
                if (line.find("joint_names:") != std::string::npos) {
                    neura_motion_planning_challenge::TrajectoryParser::parseJointNames(line, joint_traj.joint_names);
                }
                else if (line.find("- index:") != std::string::npos) {
                    neura_motion_planning_challenge::TrajectoryParser::parseWaypoint(file, line, joint_traj);
                }
            }
        }
        
        file.close();
        ROS_INFO("Successfully imported trajectory from: %s", filepath.c_str());
        ROS_INFO("  Joint waypoints loaded: %lu", joint_traj.points.size());
        
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in importTrajectoryFromFile: %s", e.what());
        throw;
    }
    
    return trajectory_data;
}
