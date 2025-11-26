#ifndef NEURA_MOTION_PLANNING_CHALLENGE_CART_MOTION_PLANNING_DATA_H
#define NEURA_MOTION_PLANNING_CHALLENGE_CART_MOTION_PLANNING_DATA_H

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>

namespace neura_motion_planning_challenge
{

/**
 * @brief Struct to represent a single Cartesian waypoint with additional data.
 *
 * This struct defines a data structure used to store a desired robot pose,
 * associated time from the start of the motion, and a header for reference.
 * It is typically used within motion planning calculations and visualizations.
 */
class CartMotionPlanningData
{
public:
  /**
   * @brief Default constructor, initializes with identity pose and zero time.
   */
  CartMotionPlanningData();

  /**
   * @brief Destructor. (Default behavior inherited)
   */
  ~CartMotionPlanningData() = default;

  /**
   * @brief Constructor with pose, time, and header.
   *
   * @param in_pose  The desired Cartesian pose of the robot.
   * @param time   The time offset from the start of the motion (in seconds).
   * @param header  A ROS header associated with the waypoint.
   */
  CartMotionPlanningData(const geometry_msgs::Pose &in_pose, const double &time, const std_msgs::Header &header);

  /**
   * @brief Gets a copy of the current CartMotionPlanningData.
   *
   * @return A deep copy of the current object's data.
   */
  CartMotionPlanningData getData(void) const;

  /**
   * @brief Get the pose.
   */
  const geometry_msgs::Pose& getPose() const { return pose; }

  /**
   * @brief Get the time from start.
   */
  const ros::Duration& getTimeFromStart() const { return time_from_start; }

  /**
   * @brief Get the header.
   */
  const std_msgs::Header& getHeader() const { return header; }

private:
  /**
   * @brief The desired Cartesian pose of the robot.
   */
  geometry_msgs::Pose pose;

  /**
   * @brief The time offset from the start of the motion (in seconds).
   */
  ros::Duration time_from_start;

  /**
   * @brief A ROS header associated with the waypoint.
   */
  std_msgs::Header header;
};

} // namespace neura_motion_planning_challenge

#endif // NEURA_MOTION_PLANNING_CHALLENGE_CART_MOTION_PLANNING_DATA_H