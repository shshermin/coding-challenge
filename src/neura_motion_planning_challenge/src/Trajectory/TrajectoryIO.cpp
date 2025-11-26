#include <neura_motion_planning_challenge/TrajectoryIO.h>

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
