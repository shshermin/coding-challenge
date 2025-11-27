#ifndef NEURA_MOTION_PLANNING_CHALLENGE_TRAJECTORY_IO_H
#define NEURA_MOTION_PLANNING_CHALLENGE_TRAJECTORY_IO_H

#include <trajectory_msgs/JointTrajectory.h>
#include <neura_motion_planning_challenge/DataStructure/TrajectoryData.h>
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

   /**
   * @brief Exports a robot trajectory to a file.
   *
   * This function writes the provided joint trajectory (`joint_trajectory`) and an optional set of corresponding Cartesian waypoints (`cart_pose_array`) to a file specified by `filepath`.
   * If successful, it returns `true`. Otherwise, it returns `false` and sets an error message.
   *
   * @param filepath  The path to the file where the trajectory will be saved.
   * @param joint_trajectory  The planned joint trajectory to be exported, represented as a `trajectory_msgs::JointTrajectory` message.
   * @param cart_pose_array  (Optional) A vector of `CartMotionPlanningData` structs, corresponding to the waypoints of the joint trajectory.
   *                        If provided, these waypoints will be included in the output file for additional reference.
   *
   * @return `true` if the trajectory was successfully exported, `false` otherwise.
   *
   * @throws std::runtime_error  If an error occurs during file writing, a detailed error
   *                            message is thrown.
   */
  static bool exportTrajectoryToFile(const std::string &filepath, const trajectory_msgs::JointTrajectory &joint_trajectory, const std::vector<CartMotionPlanningData> &cart_pose_array);

  /**
   * @brief Imports a robot trajectory from a file and returns it as a TrajectoryData object.
   *
   * This function attempts to read and parse a robot trajectory from the specified file.
   * If successful, it returns a `TrajectoryData` object containing the imported data.
   * Otherwise, it throws an exception indicating the error.
   *
   * @param filepath  The path to the file containing the trajectory data.
   *
   * @return A `TrajectoryData` object holding the imported trajectory information.
   *
   * @throws std::runtime_error  If an error occurs during file reading or parsing,
   *                             a detailed error message is thrown.
   *
   * @note The specific format of the input file may vary depending on the implementation.
   *       Consult the class documentation for supported formats and the structure of the
   *       returned `TrajectoryData` object.
   *       Additionally, this function might require appropriate permissions to read the
   *       specified file.
   */
  static TrajectoryData importTrajectoryFromFile(const std::string &filepath);
};

} // namespace neura_motion_planning_challenge

#endif // NEURA_MOTION_PLANNING_CHALLENGE_TRAJECTORY_IO_H
