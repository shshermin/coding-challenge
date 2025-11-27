#ifndef NEURA_MOTION_PLANNING_CHALLENGE_PLAN_METADATA_H
#define NEURA_MOTION_PLANNING_CHALLENGE_PLAN_METADATA_H

#include <string>
#include <vector>
#include <trajectory_msgs/JointTrajectory.h>
#include <neura_motion_planning_challenge/DataStructure/PlanCriteria.h>
#include <moveit/planning_scene/planning_scene.h>

namespace neura_motion_planning_challenge
{

class PlanMetadata {
private:
    double length_;
    std::string trajectory_filepath_;
    double plan_time_;
    double sum_abs_joints_;
    std::string planner_id_;
    double peak_velocity_;
    double avg_velocity_;

public:
    PlanMetadata();
    PlanMetadata(const std::string& file, double plan_time);
    PlanMetadata(const std::string& file, double plan_time, const std::string& planner_id);

    // Setters
    void setTrajectoryFile(const std::string& file);
    void setPlanTime(double t);
    void setPlannerId(const std::string& id);

    // Getters
    double getLength() const;
    std::string getTrajectoryFile() const;
    double getPlanTime() const;
    double getSumAbsJoints() const;
    std::string getPlannerId() const;
    double getPeakVelocity() const;
    double getAvgVelocity() const;

    // Calculation methods
    void calculateLength(const trajectory_msgs::JointTrajectory& traj);
    void calculateSumAbsJoints(const trajectory_msgs::JointTrajectory& traj);
    void calculateVelocities(const trajectory_msgs::JointTrajectory& traj);
    
    // Calculate all metrics from trajectory in one call
    void calculateAllMetrics(const trajectory_msgs::JointTrajectory& traj,
                             double plan_time);

    // Get value for a specific criteria
    double getCriteriaValue(PlanEvaluationCriteria criteria) const;



};

} // namespace neura_motion_planning_challenge

#endif // NEURA_MOTION_PLANNING_CHALLENGE_PLAN_METADATA_H
