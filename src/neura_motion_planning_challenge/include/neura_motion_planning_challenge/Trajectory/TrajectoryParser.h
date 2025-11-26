#ifndef NEURA_MOTION_PLANNING_CHALLENGE_TRAJECTORY_PARSER_H
#define NEURA_MOTION_PLANNING_CHALLENGE_TRAJECTORY_PARSER_H

#include <trajectory_msgs/JointTrajectory.h>
#include <string>
#include <vector>
#include <fstream>

namespace neura_motion_planning_challenge
{

/**
 * @class TrajectoryParser
 * @brief Static utility class for parsing trajectory data from strings and files.
 *
 * This class provides methods for parsing trajectory data from various formats
 * including custom YAML-like format and JSON.
 *
 * @note All methods are static. No instantiation required - use TrajectoryParser::methodName().
 */
class TrajectoryParser
{
public:
  // Delete constructors to enforce static-only usage
  TrajectoryParser() = delete;
  TrajectoryParser(const TrajectoryParser&) = delete;
  TrajectoryParser& operator=(const TrajectoryParser&) = delete;

  /**
   * @brief Helper to parse joint names from import file.
   * @param line Input line containing joint names.
   * @param joint_names Output vector to store parsed joint names.
   */
  static void parseJointNames(const std::string& line, std::vector<std::string>& joint_names);
  
  /**
   * @brief Helper to parse a waypoint from import file.
   * @param file Input file stream.
   * @param current_line Current line being processed.
   * @param joint_traj Joint trajectory to append the waypoint to.
   */
  static void parseWaypoint(std::ifstream& file, std::string& current_line, trajectory_msgs::JointTrajectory& joint_traj);
  
  /**
   * @brief Helper to parse a vector of doubles from a string like "[1.0, 2.0, 3.0]".
   * @param str Input string containing the vector.
   * @return Parsed vector of doubles.
   */
  static std::vector<double> parseVector(const std::string& str);
  
  /**
   * @brief Helper to parse JSON trajectory format and convert to JointTrajectory.
   * @param filepath Path to the JSON file.
   * @param joint_traj Output joint trajectory.
   * @return true if parsing succeeded, false otherwise.
   */
  static bool parseJsonTrajectory(const std::string& filepath, trajectory_msgs::JointTrajectory& joint_traj);
};

} // namespace neura_motion_planning_challenge

#endif // NEURA_MOTION_PLANNING_CHALLENGE_TRAJECTORY_PARSER_H
