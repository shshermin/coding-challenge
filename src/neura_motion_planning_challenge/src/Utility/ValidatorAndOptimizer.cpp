#include <neura_motion_planning_challenge/Utility/ValidatorAndOptimizer.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/joint_model.h>
#include <moveit/planning_scene/planning_scene.h>
#include <limits>
#include <vector>
#include <ros/ros.h>

using namespace neura_motion_planning_challenge;

bool TrajectoryValidatorAndOptimizer::validateTrajectory(const trajectory_msgs::JointTrajectory& traj,
                                       const std::string& group_name,
                                       const planning_scene::PlanningScenePtr& scene,
                                       std::string* error_out) {
    std::vector<std::string> errors;
    bool all_valid = true;
    
    if (traj.joint_names.empty() || traj.points.empty()) {
        errors.push_back("Empty joint_names or points");
        if (error_out) *error_out = errors[0];
        return false;
    }

    planning_scene::PlanningScenePtr ps = scene;
    moveit::core::RobotModelConstPtr model;
    if (!ps) {
        robot_model_loader::RobotModelLoader loader("robot_description");
        model = loader.getModel();
        if (!model) {
            errors.push_back("Failed to load robot model from robot_description");
            if (error_out) *error_out = errors[0];
            return false;
        }
        ps = std::make_shared<planning_scene::PlanningScene>(model);
    } else {
        model = ps->getRobotModel();
    }

    const moveit::core::JointModelGroup* jmg = nullptr;
    if (!group_name.empty()) {
        jmg = model->getJointModelGroup(group_name);
        if (!jmg) {
            errors.push_back(std::string("Unknown planning group: ") + group_name);
            if (error_out) *error_out = errors[0];
            return false;
        }
    }

    moveit::core::RobotState state(model);
    state.setToDefaultValues();

    std::vector<double> vel_limits(traj.joint_names.size(), std::numeric_limits<double>::infinity());
    for (size_t j = 0; j < traj.joint_names.size(); ++j) {
        const moveit::core::JointModel* jm = model->getJointModel(traj.joint_names[j]);
        if (!jm) continue;
        const std::vector<std::string>& vars = jm->getVariableNames();
        if (vars.empty()) continue;
        const auto& b = model->getVariableBounds(vars[0]);
        if (b.velocity_bounded_) vel_limits[j] = b.max_velocity_;
    }

    int bounds_violations = 0;
    int collision_violations = 0;
    int velocity_violations = 0;
    int size_mismatches = 0;
    
    std::vector<double> prev_pos;
    double prev_time = 0.0;
    bool have_prev = false;

    for (size_t i = 0; i < traj.points.size(); ++i) {
        const auto& p = traj.points[i];
        if (p.positions.size() != traj.joint_names.size()) {
            size_mismatches++;
            all_valid = false;
            continue;
        }

        state.setVariablePositions(traj.joint_names, p.positions);
        state.update();

        bool bounds_ok = jmg ? state.satisfiesBounds(jmg) : state.satisfiesBounds();
        if (!bounds_ok) {
            bounds_violations++;
            all_valid = false;
        }

        if (ps->isStateColliding(state, group_name)) {
            collision_violations++;
            all_valid = false;
        }

        if (have_prev) {
            const double dt = p.time_from_start.toSec() - prev_time;
            if (dt > 0.0) {
                for (size_t j = 0; j < prev_pos.size(); ++j) {
                    const double v = std::abs((p.positions[j] - prev_pos[j]) / dt);
                    if (v > vel_limits[j]) {
                        velocity_violations++;
                        all_valid = false;
                        break;
                    }
                }
            }
        }

        prev_pos = p.positions;
        prev_time = p.time_from_start.toSec();
        have_prev = true;
    }

    if (size_mismatches > 0) errors.push_back(std::to_string(size_mismatches) + " waypoint(s) with mismatched positions size");
    if (bounds_violations > 0) errors.push_back(std::to_string(bounds_violations) + " waypoint(s) violate joint position bounds");
    if (collision_violations > 0) errors.push_back(std::to_string(collision_violations) + " waypoint(s) in collision");
    if (velocity_violations > 0) errors.push_back(std::to_string(velocity_violations) + " waypoint(s) violate velocity limits");

    if (all_valid) {
        ROS_INFO("✓ Trajectory validation PASSED");
        ROS_INFO("  - All %lu waypoints are valid", traj.points.size());
        ROS_INFO("  - Joint position bounds: OK");
        ROS_INFO("  - Collision-free: OK");
        ROS_INFO("  - Velocity limits: OK");
    } else {
        ROS_ERROR("✗ Trajectory validation FAILED");
        ROS_ERROR("  Total waypoints: %lu", traj.points.size());
        ROS_ERROR("  - Size mismatches: %d waypoint(s)", size_mismatches);
        ROS_ERROR("  - Joint bounds violations: %d waypoint(s)", bounds_violations);
        ROS_ERROR("  - Collision violations: %d waypoint(s)", collision_violations);
        ROS_ERROR("  - Velocity violations: %d waypoint(s)", velocity_violations);
    }

    if (error_out && !errors.empty()) {
        std::string combined;
        for (size_t i = 0; i < errors.size(); ++i) {
            combined += errors[i];
            if (i < errors.size() - 1) combined += "; ";
        }
        *error_out = combined;
    }

    return all_valid;
}

// TODO: Implement optimizeTrajectory - currently not implemented
bool TrajectoryValidatorAndOptimizer::optimizeTrajectory(trajectory_msgs::JointTrajectory& trajectory, 
                                                          const std::string& group_name) {
    ROS_WARN("TrajectoryValidatorAndOptimizer::optimizeTrajectory() not yet implemented");
    return false;
}

// TODO: Implement plotTrajectoryComparison - currently not implemented
bool TrajectoryValidatorAndOptimizer::plotTrajectoryComparison(const trajectory_msgs::JointTrajectory& original_traj,
                                                                 const trajectory_msgs::JointTrajectory& optimized_traj,
                                                                 const std::string& output_path) {
    ROS_WARN("TrajectoryValidatorAndOptimizer::plotTrajectoryComparison() not yet implemented");
    return false;
}

bool TrajectoryValidatorAndOptimizer::isJointLimitsValid(const std::vector<double>& joint_config,
                                                           const std::vector<std::string>& joint_names,
                                                           const std::string& group_name) {
    if (joint_config.empty() || joint_names.empty()) {
        return false;
    }
    
    if (joint_config.size() != joint_names.size()) {
        ROS_ERROR("Joint config size (%zu) does not match joint_names size (%zu)",
                  joint_config.size(), joint_names.size());
        return false;
    }
    
    try {
        robot_model_loader::RobotModelLoader loader("robot_description");
        moveit::core::RobotModelConstPtr model = loader.getModel();
        if (!model) {
            ROS_ERROR("Failed to load robot model");
            return false;
        }
        
        moveit::core::RobotState state(model);
        state.setVariablePositions(joint_names, joint_config);
        state.update();
        
        const moveit::core::JointModelGroup* jmg = model->getJointModelGroup(group_name);
        if (jmg) {
            return state.satisfiesBounds(jmg);
        } else {
            return state.satisfiesBounds();
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in isJointLimitsValid: %s", e.what());
        return false;
    }
}

bool TrajectoryValidatorAndOptimizer::isCollisionFree(const std::vector<double>& joint_config,
                                                        const std::vector<std::string>& joint_names,
                                                        const std::string& group_name,
                                                        const planning_scene::PlanningScenePtr& planning_scene) {
    if (joint_config.empty() || joint_names.empty()) {
        return false;
    }
    
    if (joint_config.size() != joint_names.size()) {
        ROS_ERROR("Joint config size (%zu) does not match joint_names size (%zu)",
                  joint_config.size(), joint_names.size());
        return false;
    }
    
    try {
        robot_model_loader::RobotModelLoader loader("robot_description");
        moveit::core::RobotModelConstPtr model = loader.getModel();
        if (!model) {
            ROS_ERROR("Failed to load robot model");
            return false;
        }
        
        planning_scene::PlanningScenePtr ps = planning_scene;
        if (!ps) {
            ps = std::make_shared<planning_scene::PlanningScene>(model);
        }
        
        moveit::core::RobotState state(model);
        state.setVariablePositions(joint_names, joint_config);
        state.update();
        
        return !ps->isStateColliding(state, group_name);
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in isCollisionFree: %s", e.what());
        return false;
    }
}

bool TrajectoryValidatorAndOptimizer::isPathCollisionFree(const std::vector<double>& config1,
                                                            const std::vector<double>& config2,
                                                            const std::vector<std::string>& joint_names,
                                                            const std::string& group_name,
                                                            const planning_scene::PlanningScenePtr& planning_scene,
                                                            int num_samples) {
    if (config1.empty() || config2.empty() || joint_names.empty()) {
        return false;
    }
    
    if (config1.size() != config2.size() || config1.size() != joint_names.size()) {
        ROS_ERROR("Config sizes do not match joint_names size");
        return false;
    }
    
    if (num_samples < 2) num_samples = 2;
    
    try {
        robot_model_loader::RobotModelLoader loader("robot_description");
        moveit::core::RobotModelConstPtr model = loader.getModel();
        if (!model) {
            ROS_ERROR("Failed to load robot model");
            return false;
        }
        
        planning_scene::PlanningScenePtr ps = planning_scene;
        if (!ps) {
            ps = std::make_shared<planning_scene::PlanningScene>(model);
        }
        
        // Check interpolated configurations along the path
        for (int i = 0; i < num_samples; ++i) {
            double t = static_cast<double>(i) / (num_samples - 1);
            
            std::vector<double> interpolated_config(config1.size());
            for (size_t j = 0; j < config1.size(); ++j) {
                interpolated_config[j] = config1[j] * (1.0 - t) + config2[j] * t;
            }
            
            moveit::core::RobotState state(model);
            state.setVariablePositions(joint_names, interpolated_config);
            state.update();
            
            if (ps->isStateColliding(state, group_name)) {
                return false;  // Path is in collision
            }
        }
        
        return true;  // Entire path is collision-free
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in isPathCollisionFree: %s", e.what());
        return false;
    }
}
