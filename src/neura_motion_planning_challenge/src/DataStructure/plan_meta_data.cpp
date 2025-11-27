#include <neura_motion_planning_challenge/DataStructure/PlanMetaData.h>

using namespace neura_motion_planning_challenge;
#include <neura_motion_planning_challenge/Planning/motion_planning.h>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <stdexcept>


#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/joint_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <limits>

PlanMetadata::PlanMetadata() : length_(0.0), trajectory_filepath_(""), plan_time_(0.0), sum_abs_joints_(0.0), planner_id_(""), peak_velocity_(0.0), avg_velocity_(0.0) {}

PlanMetadata::PlanMetadata(const std::string& file, double plan_time) : length_(0.0), trajectory_filepath_(file), plan_time_(plan_time), sum_abs_joints_(0.0), planner_id_(""), peak_velocity_(0.0), avg_velocity_(0.0) {}

PlanMetadata::PlanMetadata(const std::string& file, double plan_time, const std::string& planner_id) : length_(0.0), trajectory_filepath_(file), plan_time_(plan_time), sum_abs_joints_(0.0), planner_id_(planner_id), peak_velocity_(0.0), avg_velocity_(0.0) {}

void PlanMetadata::setTrajectoryFile(const std::string& file) {
    trajectory_filepath_ = file;
}

void PlanMetadata::setPlanTime(double t) {
    plan_time_ = t;
}

void PlanMetadata::setPlannerId(const std::string& id) {
    planner_id_ = id;
}

double PlanMetadata::getLength() const {
    return length_;
}

std::string PlanMetadata::getTrajectoryFile() const {
    return trajectory_filepath_;
}

double PlanMetadata::getPlanTime() const {
    return plan_time_;
}

double PlanMetadata::getSumAbsJoints() const {
    return sum_abs_joints_;
}

std::string PlanMetadata::getPlannerId() const {
    return planner_id_;
}

double PlanMetadata::getPeakVelocity() const {
    return peak_velocity_;
}

double PlanMetadata::getAvgVelocity() const {
    return avg_velocity_;
}

// Calculate trajectory length in joint space calculated by summing Euclidean distances between waypoints
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

// Calculate sum of absolute joint positions --> trajectory effort
void PlanMetadata::calculateSumAbsJoints(const trajectory_msgs::JointTrajectory& traj) {
    double total = 0.0;
    for (const auto& point : traj.points) {
        for (double pos : point.positions) {
            total += std::abs(pos);
        }
    }
    sum_abs_joints_ = total;
}

// Calculate peak and average velocities from trajectory
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

// Calculate all metrics from trajectory in one call
void PlanMetadata::calculateAllMetrics(const trajectory_msgs::JointTrajectory& traj,
                                        double plan_time) {
    calculateLength(traj);
    calculateSumAbsJoints(traj);
    calculateVelocities(traj);
    setPlanTime(plan_time);
}

// Get value for a specific criteria
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
