#include <neura_motion_planning_challenge/Planning/PlanMetaData.h>

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

PlanMetadata::PlanMetadata() : length_(0.0), trajectory_file_(""), plan_time_(0.0), sum_abs_joints_(0.0), planner_id_("") {}

PlanMetadata::PlanMetadata(const std::string& file, double plan_time) : length_(0.0), trajectory_file_(file), plan_time_(plan_time), sum_abs_joints_(0.0), planner_id_("") {}

PlanMetadata::PlanMetadata(const std::string& file, double plan_time, const std::string& planner_id) : length_(0.0), trajectory_file_(file), plan_time_(plan_time), sum_abs_joints_(0.0), planner_id_(planner_id) {}

void PlanMetadata::setTrajectoryFile(const std::string& file) {
    trajectory_file_ = file;
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
    return trajectory_file_;
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

// Get value for a specific criteria
double PlanMetadata::getCriteriaValue(PlanCriteria criteria) const {
    switch (criteria) {
        case PlanCriteria::LENGTH:
            return getLength();
        case PlanCriteria::SUM_ABS_JOINTS:
            return getSumAbsJoints();
        case PlanCriteria::PLAN_TIME:
            return getPlanTime();
        default:
            return 0.0;
    }
}

// (Comparison and validation moved to PlanComparator / PlanValidator)
