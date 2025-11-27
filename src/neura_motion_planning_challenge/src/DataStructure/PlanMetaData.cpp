#include <neura_motion_planning_challenge/DataStructure/PlanMetaData.h>

using namespace neura_motion_planning_challenge;
#include <neura_motion_planning_challenge/Planning/MotionPlanning.h>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <stdexcept>


#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/joint_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <limits>

/**
 * @brief Default constructor for PlanMetadata.
 * 
 * Initializes all metrics to default values (0.0 or empty strings).
 */
PlanMetadata::PlanMetadata() : length_(0.0), trajectory_filepath_(""), plan_time_(0.0), sum_abs_joints_(0.0), planner_id_(""), peak_velocity_(0.0), avg_velocity_(0.0) {}

/**
 * @brief Constructor with trajectory file and plan time.
 * 
 * @param file Path to the trajectory file.
 * @param plan_time The time taken for planning (in seconds).
 */
PlanMetadata::PlanMetadata(const std::string& file, double plan_time) : length_(0.0), trajectory_filepath_(file), plan_time_(plan_time), sum_abs_joints_(0.0), planner_id_(""), peak_velocity_(0.0), avg_velocity_(0.0) {}

/**
 * @brief Constructor with trajectory file, plan time, and planner ID.
 * 
 * @param file Path to the trajectory file.
 * @param plan_time The time taken for planning (in seconds).
 * @param planner_id The name of the planner algorithm used (e.g., "RRTConnect", "RRTstar").
 */
PlanMetadata::PlanMetadata(const std::string& file, double plan_time, const std::string& planner_id) : length_(0.0), trajectory_filepath_(file), plan_time_(plan_time), sum_abs_joints_(0.0), planner_id_(planner_id), peak_velocity_(0.0), avg_velocity_(0.0) {}

/**
 * @brief Sets the trajectory file path.
 * 
 * @param file Path to the trajectory file to set.
 */
void PlanMetadata::setTrajectoryFile(const std::string& file) {
    trajectory_filepath_ = file;
}

/**
 * @brief Sets the planning time.
 * 
 * @param t The planning time in seconds.
 */
void PlanMetadata::setPlanTime(double t) {
    plan_time_ = t;
}

/**
 * @brief Sets the planner algorithm ID.
 * 
 * @param id The planner algorithm ID (e.g., "RRTConnect", "RRTstar", "PRM").
 */
void PlanMetadata::setPlannerId(const std::string& id) {
    planner_id_ = id;
}

/**
 * @brief Gets the trajectory length.
 * 
 * @return The trajectory length in joint space (sum of Euclidean distances between waypoints).
 */
double PlanMetadata::getLength() const {
    return length_;
}

/**
 * @brief Gets the trajectory file path.
 * 
 * @return The path to the trajectory file.
 */
std::string PlanMetadata::getTrajectoryFile() const {
    return trajectory_filepath_;
}

/**
 * @brief Gets the planning time.
 * 
 * @return The planning time in seconds.
 */
double PlanMetadata::getPlanTime() const {
    return plan_time_;
}

/**
 * @brief Gets the sum of absolute joint positions.
 * 
 * @return The sum of absolute joint movements across all waypoints.
 */
double PlanMetadata::getSumAbsJoints() const {
    return sum_abs_joints_;
}

/**
 * @brief Gets the planner algorithm ID.
 * 
 * @return The planner algorithm ID used for this plan.
 */
std::string PlanMetadata::getPlannerId() const {
    return planner_id_;
}

/**
 * @brief Gets the peak velocity of the trajectory.
 * 
 * @return The maximum velocity magnitude encountered in the trajectory.
 */
double PlanMetadata::getPeakVelocity() const {
    return peak_velocity_;
}

/**
 * @brief Gets the average velocity of the trajectory.
 * 
 * @return The average velocity across all waypoints.
 */
double PlanMetadata::getAvgVelocity() const {
    return avg_velocity_;
}

/**
 * @brief Calculates the trajectory length in joint space.
 * 
 * Computes the length by summing Euclidean distances between consecutive waypoints.
 * 
 * @param traj The joint trajectory to analyze.
 */
void PlanMetadata::calculateLength(const trajectory_msgs::JointTrajectory& traj) {
    double total = 0.0;
    for (size_t i = 1; i < traj.points.size(); ++i) {
        double dist = 0.0;
        for (size_t j = 0; j < traj.points[i].positions.size(); ++j) {
            double diff = traj.points[i].positions[j] - traj.points[i-1].positions[j];
            dist += diff * diff;
        }
        total += std::sqrt(dist);
    }
    length_ = total;
}

/**
 * @brief Calculates the sum of absolute joint movements.
 * 
 * Computes the trajectory effort by summing the absolute value of all joint positions.
 * 
 * @param traj The joint trajectory to analyze.
 */
void PlanMetadata::calculateSumAbsJoints(const trajectory_msgs::JointTrajectory& traj) {
    double total = 0.0;
    for (const auto& point : traj.points) {
        for (double pos : point.positions) {
            total += std::abs(pos);
        }
    }
    sum_abs_joints_ = total;
}

/**
 * @brief Calculates peak and average velocities from the trajectory.
 * 
 * Computes velocity metrics by analyzing distance traveled between consecutive waypoints
 * and dividing by the time difference.
 * 
 * @param traj The joint trajectory to analyze.
 */
void PlanMetadata::calculateVelocities(const trajectory_msgs::JointTrajectory& traj) {
    peak_velocity_ = 0.0;
    avg_velocity_ = 0.0;

    if (traj.points.size() < 2) {
        return;
    }

    double total_distance = 0.0;
    double total_time = 0.0;

    for (size_t i = 1; i < traj.points.size(); ++i) {
        const auto& prev = traj.points[i - 1];
        const auto& curr = traj.points[i];

        // Calculate time difference
        double dt = curr.time_from_start.toSec() - prev.time_from_start.toSec();
        
        if (dt <= 0.0) continue;

        // Calculate Euclidean distance between waypoints
        double dist = 0.0;
        for (size_t j = 0; j < prev.positions.size() && j < curr.positions.size(); ++j) {
            double dq = curr.positions[j] - prev.positions[j];
            dist += dq * dq;
        }
        dist = std::sqrt(dist);

        // Calculate velocity for this segment
        double segment_velocity = dist / dt;
        peak_velocity_ = std::max(peak_velocity_, segment_velocity);

        total_distance += dist;
        total_time += dt;
    }

    // Calculate average velocity
    if (total_time > 0.0) {
        avg_velocity_ = total_distance / total_time;
    }
}

/**
 * @brief Calculates all trajectory metrics at once.
 * 
 * Computes length, sum of absolute joints, velocities, and sets the plan time.
 * 
 * @param traj The joint trajectory to analyze.
 * @param plan_time The planning time in seconds.
 */
void PlanMetadata::calculateAllMetrics(const trajectory_msgs::JointTrajectory& traj,
                                        double plan_time) {
    calculateLength(traj);
    calculateSumAbsJoints(traj);
    calculateVelocities(traj);
    setPlanTime(plan_time);
}

/**
 * @brief Gets the value for a specific evaluation criteria.
 * 
 * @param criteria The evaluation criteria (LENGTH, SUM_ABS_JOINTS, or PLAN_TIME).
 * @return The numerical value corresponding to the specified criteria.
 */
double PlanMetadata::getCriteriaValue(PlanEvaluationCriteria criteria) const {
    switch(criteria) {
        case PlanEvaluationCriteria::LENGTH:
            return getLength();
        case PlanEvaluationCriteria::SUM_ABS_JOINTS:
            return getSumAbsJoints();
        case PlanEvaluationCriteria::PLAN_TIME:
            return getPlanTime();
        default:
            return 0.0;
    }
}

// (Comparison and validation moved to PlanComparator / PlanValidator)
