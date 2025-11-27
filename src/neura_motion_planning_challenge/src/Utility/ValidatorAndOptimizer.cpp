#include <neura_motion_planning_challenge/Utility/ValidatorAndOptimizer.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/joint_model.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <limits>
#include <vector>
#include <ros/ros.h>
#include <cmath>
#include <fstream>
#include <sstream>

using namespace neura_motion_planning_challenge;

double TrajectoryValidatorAndOptimizer::computeVelocity(const std::vector<double>& positions1,
                                                        const std::vector<double>& positions2,
                                                        double time_diff) {
    if (positions1.empty() || positions2.empty() || time_diff <= 0.0) {
        return 0.0;
    }
    
    if (positions1.size() != positions2.size()) {
        ROS_WARN("computeVelocity: Position vectors have different sizes");
        return 0.0;
    }
    
    // Compute Euclidean distance in configuration space
    double distance = 0.0;
    for (size_t i = 0; i < positions1.size(); ++i) {
        double delta = positions2[i] - positions1[i];
        distance += delta * delta;
    }
    distance = std::sqrt(distance);
    
    // Return velocity magnitude
    return distance / time_diff;
}


bool TrajectoryValidatorAndOptimizer::validateTrajectory(const trajectory_msgs::JointTrajectory& traj,
                                       const std::string& group_name,
                                       const planning_scene::PlanningScenePtr& scene,
                                       std::string* error_out,
                                       const moveit::core::RobotModelConstPtr& robot_model_in) {
    std::vector<std::string> errors;
    bool all_valid = true;
    
    if (traj.joint_names.empty() || traj.points.empty()) {
        errors.push_back("Empty joint_names or points");
        if (error_out) *error_out = errors[0];
        return false;
    }

    planning_scene::PlanningScenePtr ps = scene;
    moveit::core::RobotModelConstPtr model = robot_model_in;
    if (!model) {
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
    } else if (!ps) {
        ps = std::make_shared<planning_scene::PlanningScene>(model);
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
                                                          const std::string& group_name,
                                                          bool use_time_optimal_trajectory_generation,
                                                          bool use_iterative_parabolic,
                                                          const moveit::core::RobotModelConstPtr& robot_model_in) {
    try {
        // Validate that at least one method is selected
        if (!use_time_optimal_trajectory_generation && !use_iterative_parabolic) {
            ROS_WARN("optimizeTrajectory: No optimization methods selected. Set at least one flag to true.");
            return false;
        }

        // Load robot model if not provided
        moveit::core::RobotModelConstPtr robot_model = robot_model_in;
        if (!robot_model) {
            robot_model_loader::RobotModelLoader loader("robot_description");
            robot_model = loader.getModel();
            if (!robot_model) {
                ROS_ERROR("Failed to load robot model for trajectory optimization");
                return false;
            }
        }

        // Get the planning group
        const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(group_name);
        if (!jmg) {
            ROS_ERROR("Unknown planning group: %s", group_name.c_str());
            return false;
        }

        // Convert trajectory_msgs::JointTrajectory to robot_trajectory::RobotTrajectory
        robot_trajectory::RobotTrajectory robot_traj(robot_model, group_name);
        moveit::core::RobotState state(robot_model);
        state.setToDefaultValues();

        for (const auto& point : trajectory.points) {
            state.setVariablePositions(trajectory.joint_names, point.positions);
            robot_traj.addSuffixWayPoint(state, 0.0);  // Add with 0 time, will be updated
        }

        bool any_success = false;
        ROS_INFO("Starting trajectory optimization...");

        // Apply TimeOptimalTrajectoryGeneration if selected
        if (use_time_optimal_trajectory_generation) {
            try {
                trajectory_processing::TimeOptimalTrajectoryGeneration totg;
                bool success = totg.computeTimeStamps(robot_traj, 0.1, 0.1);
                
                if (success) {
                    ROS_INFO("  ✓ TimeOptimalTrajectoryGeneration PASSED");
                    any_success = true;
                } else {
                    ROS_WARN("  ✗ TimeOptimalTrajectoryGeneration failed");
                }
            } catch (const std::exception& e) {
                ROS_WARN("  ✗ TimeOptimalTrajectoryGeneration exception: %s", e.what());
            }
        }

        // Apply IterativeParabolicTimeParameterization if selected
        if (use_iterative_parabolic) {
            try {
                trajectory_processing::IterativeParabolicTimeParameterization iptp;
                bool success = iptp.computeTimeStamps(robot_traj, 0.1, 0.1);
                
                if (success) {
                    ROS_INFO("  ✓ IterativeParabolicTimeParameterization PASSED");
                    any_success = true;
                } else {
                    ROS_WARN("  ✗ IterativeParabolicTimeParameterization failed");
                }
            } catch (const std::exception& e) {
                ROS_WARN("  ✗ IterativeParabolicTimeParameterization exception: %s", e.what());
            }
        }

        if (any_success) {
            // Convert back to trajectory_msgs::JointTrajectory
            trajectory.points.clear();

            for (size_t i = 0; i < robot_traj.getWayPointCount(); ++i) {
                trajectory_msgs::JointTrajectoryPoint point;
                const auto& waypoint = robot_traj.getWayPoint(i);
                
                // Copy positions
                const double* pos = waypoint.getVariablePositions();
                point.positions.assign(pos, pos + jmg->getVariableCount());
                
                // Copy velocities
                const double* vel = waypoint.getVariableVelocities();
                point.velocities.assign(vel, vel + jmg->getVariableCount());
                
                // Copy accelerations
                const double* accel = waypoint.getVariableAccelerations();
                point.accelerations.assign(accel, accel + jmg->getVariableCount());
                
                point.time_from_start = ros::Duration(robot_traj.getWayPointDurationFromStart(i));
                
                trajectory.points.push_back(point);
            }

            ROS_INFO("✓ Trajectory optimization SUCCESSFUL");
            ROS_INFO("  - Velocity & acceleration profiles added");
            ROS_INFO("  - Waypoints: %lu", trajectory.points.size());
            
            if (!trajectory.points.empty()) {
                ROS_INFO("  - Duration: %.3f seconds", 
                         trajectory.points.back().time_from_start.toSec());
            }
        } else {
            ROS_ERROR("✗ All selected optimization methods failed");
        }
        
        return any_success;

    } catch (const std::exception& e) {
        ROS_ERROR("Exception in optimizeTrajectory: %s", e.what());
        return false;
    }
}

// Generate trajectory comparison plot using PlanMetadata objects
bool TrajectoryValidatorAndOptimizer::generateTrajectoryComparisonPlot(
    const PlanMetadata& original_plan,
    const PlanMetadata& optimized_plan,
    const std::string& output_dir) {
    
    try {
        std::string plot_script_path = output_dir + "/trajectory_comparison.py";
        std::ofstream plot_script(plot_script_path);
        
        if (!plot_script.is_open()) {
            ROS_ERROR("Failed to create plot script at %s", plot_script_path.c_str());
            return false;
        }

        // Extract metrics from PlanMetadata objects
        double orig_duration = original_plan.getPlanTime();
        double opt_duration = optimized_plan.getPlanTime();
        double orig_length = original_plan.getLength();
        double opt_length = optimized_plan.getLength();
        double orig_sum_joints = original_plan.getSumAbsJoints();
        double opt_sum_joints = optimized_plan.getSumAbsJoints();
        double orig_peak_vel = original_plan.getPeakVelocity();
        double opt_peak_vel = optimized_plan.getPeakVelocity();
        double orig_avg_vel = original_plan.getAvgVelocity();
        double opt_avg_vel = optimized_plan.getAvgVelocity();

        // Compute improvements for display (handle division by zero)
        double duration_improvement = ((orig_duration - opt_duration) / orig_duration) * 100.0;
        double length_improvement = ((orig_length - opt_length) / orig_length) * 100.0;
        double peak_vel_improvement = (orig_peak_vel > 0.0) ? 
            ((orig_peak_vel - opt_peak_vel) / orig_peak_vel) * 100.0 : 0.0;
        double avg_vel_improvement = (orig_avg_vel > 0.0) ? 
            ((orig_avg_vel - opt_avg_vel) / orig_avg_vel) * 100.0 : 0.0;
        // For joint motion, use 0 if original is 0 (metric not applicable)
        double joints_improvement = (orig_sum_joints > 0.0) ? 
            ((orig_sum_joints - opt_sum_joints) / orig_sum_joints) * 100.0 : 0.0;

        // Write Python plotting script
        plot_script << "#!/usr/bin/env python3\n";
        plot_script << "import matplotlib\n";
        plot_script << "matplotlib.use('Agg')  # Use non-interactive backend for headless systems\n";
        plot_script << "import matplotlib.pyplot as plt\n";
        plot_script << "import numpy as np\n\n";

        // Create figure with subplots
        plot_script << "# Create figure with subplots\n";
        plot_script << "fig, axes = plt.subplots(2, 2, figsize=(14, 10))\n";
        plot_script << "plt.subplots_adjust(top=0.96)\n\n";

        // Subplot 1: Planning metrics comparison
        plot_script << "# Planning Time and Path Length with Velocities\n";
        plot_script << "ax = axes[0, 0]\n";
        plot_script << "categories = ['Plan Time (s)', 'Path Length', 'Peak Vel (r/s)', 'Avg Vel (r/s)']\n";
        plot_script << "original = [" << orig_duration << ", " << orig_length << ", " << orig_peak_vel << ", " << orig_avg_vel << "]\n";
        plot_script << "optimized = [" << opt_duration << ", " << opt_length << ", " << opt_peak_vel << ", " << opt_avg_vel << "]\n";
        plot_script << "x = np.arange(len(categories))\n";
        plot_script << "width = 0.35\n";
        plot_script << "ax.bar(x - width/2, original, width, label='Original', color='#1f77b4', alpha=0.8)\n";
        plot_script << "ax.bar(x + width/2, optimized, width, label='Optimized', color='#ff7f0e', alpha=0.8)\n";
        plot_script << "ax.set_ylabel('Value')\n";
        plot_script << "ax.set_title('Performance Metrics')\n";
        plot_script << "ax.set_xticks(x)\n";
        plot_script << "ax.set_xticklabels(categories, rotation=15, ha='right')\n";
        plot_script << "ax.legend()\n";
        plot_script << "ax.grid(True, alpha=0.3, axis='y')\n\n";

        // Subplot 2: Joint configuration metrics
        plot_script << "# Joint Configuration Metrics\n";
        plot_script << "ax = axes[0, 1]\n";
        plot_script << "categories = ['Sum Abs Joints']\n";
        plot_script << "original = [" << orig_sum_joints << "]\n";
        plot_script << "optimized = [" << opt_sum_joints << "]\n";
        plot_script << "x = np.arange(len(categories))\n";
        plot_script << "ax.bar(x - width/2, original, width, label='Original', color='#1f77b4', alpha=0.8)\n";
        plot_script << "ax.bar(x + width/2, optimized, width, label='Optimized', color='#ff7f0e', alpha=0.8)\n";
        plot_script << "ax.set_ylabel('Sum of Absolute Joint Values (rad)')\n";
        plot_script << "ax.set_title('Joint Configuration Analysis')\n";
        plot_script << "ax.set_xticks(x)\n";
        plot_script << "ax.set_xticklabels(categories)\n";
        plot_script << "ax.legend()\n";
        plot_script << "ax.grid(True, alpha=0.3)\n\n";

        // Subplot 3: Improvements (percentage)
        plot_script << "# Optimization Improvements\n";
        plot_script << "ax = axes[1, 0]\n";
        plot_script << "improvements = [" << duration_improvement << ", " << length_improvement << ", " << peak_vel_improvement << ", " << avg_vel_improvement << ", " << joints_improvement << "]\n";
        plot_script << "categories = ['Plan Time', 'Path Length', 'Peak Vel', 'Avg Vel', 'Joint Motion']\n";
        plot_script << "colors = ['#2ca02c' if x > 0 else '#d62728' for x in improvements]\n";
        plot_script << "bars = ax.bar(categories, improvements, color=colors, alpha=0.7)\n";
        plot_script << "ax.set_ylabel('Improvement (%)')\n";
        plot_script << "ax.set_title('Optimization Gains')\n";
        plot_script << "ax.axhline(y=0, color='black', linestyle='-', linewidth=0.8)\n";
        plot_script << "ax.set_xticklabels(categories, rotation=15, ha='right')\n";
        plot_script << "for bar in bars:\n";
        plot_script << "    height = bar.get_height()\n";
        plot_script << "    ax.text(bar.get_x() + bar.get_width()/2., height,\n";
        plot_script << "            f'{height:.1f}%', ha='center', va='bottom' if height > 0 else 'top', fontsize=9)\n";
        plot_script << "ax.grid(True, alpha=0.3, axis='y')\n\n";

        // Subplot 4: Summary metrics table
        plot_script << "# Summary Comparison Table\n";
        plot_script << "ax = axes[1, 1]\n";
        plot_script << "ax.axis('tight')\n";
        plot_script << "ax.axis('off')\n";
        plot_script << "summary_data = [\n";
        plot_script << "    ['Metric', 'Original', 'Optimized', 'Improvement'],\n";
        plot_script << "    ['Planning Time (s)', f'{" << orig_duration << ":.4f}', f'{" << opt_duration << ":.4f}', f'{" << duration_improvement << ":.1f}%'],\n";
        plot_script << "    ['Path Length (rad)', f'{" << orig_length << ":.6f}', f'{" << opt_length << ":.6f}', f'{" << length_improvement << ":.1f}%'],\n";
        plot_script << "    ['Peak Velocity (r/s)', f'{" << orig_peak_vel << ":.6f}', f'{" << opt_peak_vel << ":.6f}', f'{" << peak_vel_improvement << ":.1f}%'],\n";
        plot_script << "    ['Avg Velocity (r/s)', f'{" << orig_avg_vel << ":.6f}', f'{" << opt_avg_vel << ":.6f}', f'{" << avg_vel_improvement << ":.1f}%'],\n";
        plot_script << "    ['Sum Abs Joints (rad)', f'{" << orig_sum_joints << ":.4f}', f'{" << opt_sum_joints << ":.4f}', f'{" << joints_improvement << ":.1f}%'],\n";
        plot_script << "]\n";
        plot_script << "table = ax.table(cellText=summary_data, cellLoc='center', loc='center',\n";
        plot_script << "                colWidths=[0.22, 0.26, 0.26, 0.26])\n";
        plot_script << "table.auto_set_font_size(False)\n";
        plot_script << "table.set_fontsize(9)\n";
        plot_script << "table.scale(1, 1.8)\n";
        plot_script << "# Style header row\n";
        plot_script << "for i in range(4):\n";
        plot_script << "    table[(0, i)].set_facecolor('#4472C4')\n";
        plot_script << "    table[(0, i)].set_text_props(weight='bold', color='white')\n";
        plot_script << "ax.set_title('Performance Summary', pad=20, fontweight='bold')\n\n";

        plot_script << "plt.tight_layout()\n";
        plot_script << "plt.savefig('" << output_dir << "/trajectory_comparison.png', dpi=150, bbox_inches='tight')\n";
        plot_script << "print(f'Plot saved to " << output_dir << "/trajectory_comparison.png')\n";
        plot_script << "plt.close()\n";

        plot_script.close();

        // Make script executable and run it
        std::string chmod_cmd = "chmod +x " + plot_script_path;
        int chmod_result = system(chmod_cmd.c_str());
        
        if (chmod_result != 0) {
            ROS_WARN("Failed to make plot script executable");
        }

        std::string run_cmd = "python3 " + plot_script_path + " 2>/dev/null";
        int plot_result = system(run_cmd.c_str());
        
        if (plot_result == 0) {
            ROS_INFO("✓ Trajectory comparison plot saved to: %s/trajectory_comparison.png", output_dir.c_str());
            ROS_INFO("✓ Plot script saved to: %s", plot_script_path.c_str());
            ROS_INFO("  - Duration improvement: %.1f%%", duration_improvement);
            ROS_INFO("  - Path length improvement: %.1f%%", length_improvement);
            ROS_INFO("  - Joint motion improvement: %.1f%%", joints_improvement);
            return true;
        } else {
            ROS_WARN("Failed to generate plot image, but script was created at %s", plot_script_path.c_str());
            return true;  // Script creation was successful, just plot generation failed
        }

    } catch (const std::exception& e) {
        ROS_ERROR("Exception in generateTrajectoryComparisonPlot: %s", e.what());
        return false;
    }
}



bool TrajectoryValidatorAndOptimizer::isJointLimitsValid(const std::vector<double>& joint_config,
                                                           const std::vector<std::string>& joint_names,
                                                           const std::string& group_name,
                                                           const moveit::core::RobotModelConstPtr& robot_model_in) {
    if (joint_config.empty() || joint_names.empty()) {
        return false;
    }
    
    if (joint_config.size() != joint_names.size()) {
        ROS_ERROR("Joint config size (%zu) does not match joint_names size (%zu)",
                  joint_config.size(), joint_names.size());
        return false;
    }
    
    try {
        moveit::core::RobotModelConstPtr model = robot_model_in;
        if (!model) {
            robot_model_loader::RobotModelLoader loader("robot_description");
            model = loader.getModel();
            if (!model) {
                ROS_ERROR("Failed to load robot model");
                return false;
            }
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
                                                        const planning_scene::PlanningScenePtr& planning_scene,
                                                        const moveit::core::RobotModelConstPtr& robot_model_in) {
    if (joint_config.empty() || joint_names.empty()) {
        return false;
    }
    
    if (joint_config.size() != joint_names.size()) {
        ROS_ERROR("Joint config size (%zu) does not match joint_names size (%zu)",
                  joint_config.size(), joint_names.size());
        return false;
    }
    
    try {
        moveit::core::RobotModelConstPtr model = robot_model_in;
        if (!model) {
            robot_model_loader::RobotModelLoader loader("robot_description");
            model = loader.getModel();
            if (!model) {
                ROS_ERROR("Failed to load robot model");
                return false;
            }
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
                                                            int num_samples,
                                                            const moveit::core::RobotModelConstPtr& robot_model_in) {
    if (config1.empty() || config2.empty() || joint_names.empty()) {
        return false;
    }
    
    if (config1.size() != config2.size() || config1.size() != joint_names.size()) {
        ROS_ERROR("Config sizes do not match joint_names size");
        return false;
    }
    
    if (num_samples < 2) num_samples = 2;
    
    try {
        moveit::core::RobotModelConstPtr model = robot_model_in;
        if (!model) {
            robot_model_loader::RobotModelLoader loader("robot_description");
            model = loader.getModel();
            if (!model) {
                ROS_ERROR("Failed to load robot model");
                return false;
            }
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
