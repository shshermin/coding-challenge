#ifndef NEURA_MOTION_PLANNING_CHALLENGE_TRAJECTORY_IO_H
#define NEURA_MOTION_PLANNING_CHALLENGE_TRAJECTORY_IO_H

#include <trajectory_msgs/JointTrajectory.h>
#include <neura_motion_planning_challenge/DataStructure/CartMotionPlanningData.h>
#include <vector>
#include <string>
#include <iostream>

namespace neura_motion_planning_challenge
{

/**
 * @class TrajectoryIO
 * @brief Static utility class for trajectory file I/O operations.
 *
 * This class provides methods for writing trajectory data to output streams
 * and files in a structured format.
 *
 * @note All methods are static. No instantiation required - use TrajectoryIO::methodName().
 */
class TrajectoryIO
{
public:
  // Delete constructors to enforce static-only usage
  TrajectoryIO() = delete;
  TrajectoryIO(const TrajectoryIO&) = delete;
  TrajectoryIO& operator=(const TrajectoryIO&) = delete;

  /**
   * @brief Helper to write a vector of doubles as a comma-separated array.
   * @param stream Output stream to write to.
   * @param vec Vector of doubles to write.
   */
  static void writeVectorToStream(std::ostream& stream, const std::vector<double>& vec);
  
  /**
   * @brief Helper to write joint trajectory waypoints to file.
   * @param file Output file stream.
   * @param joint_trajectory The trajectory to write.
   */
  static void writeJointTrajectory(std::ostream& file, const trajectory_msgs::JointTrajectory& joint_trajectory);
  
  /**
   * @brief Helper to write Cartesian poses to file.
   * @param file Output file stream.
   * @param cart_pose_array Array of Cartesian poses to write.
   */
  static void writeCartesianPoses(std::ostream& file, const std::vector<CartMotionPlanningData>& cart_pose_array);
};

} // namespace neura_motion_planning_challenge

#endif // NEURA_MOTION_PLANNING_CHALLENGE_TRAJECTORY_IO_H
