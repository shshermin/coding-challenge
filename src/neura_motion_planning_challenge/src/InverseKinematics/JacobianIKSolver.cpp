#include <neura_motion_planning_challenge/InverseKinematics/JacobianIKSolver.h>
#include <moveit/robot_state/conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/ros.h>
#include <cmath>

namespace neura_motion_planning_challenge
{

JacobianIKSolver::JacobianIKSolver(
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
    const std::string &planning_group,
    int max_iterations,
    double tolerance,
    double damping_factor,
    double step_size)
    : move_group_(move_group),
      planning_group_(planning_group),
      max_iterations_(max_iterations),
      tolerance_(tolerance),
      damping_factor_(damping_factor),
      step_size_(step_size)
{
    if (!move_group_) {
        throw std::runtime_error("MoveGroupInterface is null");
    }
    
    ROS_INFO("JacobianIKSolver initialized for group '%s' with max_iter=%d, tolerance=%.4f, damping=%.4f",
             planning_group_.c_str(), max_iterations_, tolerance_, damping_factor_);
}

/**
 * @brief Solves IK using iterative Jacobian-based numerical method.
 *
 * Algorithm:
 * 1. Initialize from current robot state
 * 2. For each iteration:
 *    a. Get current end-effector pose
 *    b. Compute Jacobian matrix
 *    c. Calculate position error
 *    d. Update joints: Δq = (J^T*J + λI)^-1 * J^T * Δx
 *    e. Check convergence
 */
bool JacobianIKSolver::solveIK(const geometry_msgs::Pose &target_pose,
                               std::vector<double> &joint_solution)
{
    try {
        // Initialize robot state from current state
        moveit::core::RobotStatePtr robot_state(
            new moveit::core::RobotState(move_group_->getRobotModel()));
        robot_state->setToDefaultValues();

        const moveit::core::JointModelGroup* joint_model_group =
            robot_state->getJointModelGroup(planning_group_);

        if (!joint_model_group) {
            ROS_ERROR("Planning group '%s' not found", planning_group_.c_str());
            return false;
        }

        std::string ee_link = move_group_->getEndEffectorLink();
        if (ee_link.empty()) {
            ROS_WARN("End effector link not set, using first link");
            ee_link = joint_model_group->getLinkModelNames().back();
        }

        // Extract target position and orientation
        Eigen::Vector3d target_pos(target_pose.position.x,
                                   target_pose.position.y,
                                   target_pose.position.z);

        tf2::Quaternion target_quat(target_pose.orientation.x,
                                    target_pose.orientation.y,
                                    target_pose.orientation.z,
                                    target_pose.orientation.w);

        ROS_DEBUG("JacobianIKSolver: Solving IK for target pose (%.3f, %.3f, %.3f)",
                  target_pos.x(), target_pos.y(), target_pos.z());

        // ===== ITERATIVE IK SOLVING =====
        for (int iteration = 0; iteration < max_iterations_; ++iteration) {
            // Get current end-effector pose
            const Eigen::Isometry3d& current_ee_transform =
                robot_state->getGlobalLinkTransform(ee_link);
            Eigen::Vector3d current_pos = current_ee_transform.translation();

            // Calculate position error
            Eigen::Vector3d pos_error = target_pos - current_pos;
            double error_norm = pos_error.norm();

            ROS_DEBUG("Iteration %d: position error = %.6f meters", iteration, error_norm);

            // Check convergence
            if (error_norm < tolerance_) {
                ROS_INFO("JacobianIKSolver: Converged in %d iterations with error %.6f",
                         iteration, error_norm);
                
                // Extract joint configuration
                robot_state->copyJointGroupPositions(joint_model_group, joint_solution);
                return true;
            }

            // Compute Jacobian matrix
            Eigen::MatrixXd jacobian;
            if (!computeJacobian(robot_state, jacobian)) {
                ROS_WARN("Failed to compute Jacobian at iteration %d", iteration);
                return false;
            }

            // ===== DAMPED LEAST SQUARES (Pseudo-Inverse) =====
            // Δq = (J^T*J + λI)^-1 * J^T * Δx
            
            Eigen::MatrixXd JtJ = jacobian.transpose() * jacobian;
            
            // Add damping to diagonal for numerical stability
            for (int i = 0; i < JtJ.rows(); ++i) {
                JtJ(i, i) += damping_factor_;
            }

            // Compute damped pseudo-inverse
            Eigen::MatrixXd JtJ_inv = JtJ.inverse();
            Eigen::MatrixXd J_damp_pinv = JtJ_inv * jacobian.transpose();

            // Calculate joint velocity update
            Eigen::VectorXd dq = J_damp_pinv * pos_error;

            // ===== UPDATE JOINT CONFIGURATION =====
            std::vector<double> current_joints;
            robot_state->copyJointGroupPositions(joint_model_group, current_joints);

            for (size_t i = 0; i < current_joints.size(); ++i) {
                if (i < dq.size()) {
                    current_joints[i] += step_size_ * dq(i);
                }
            }

            // Set new joint positions
            robot_state->setJointGroupPositions(joint_model_group, current_joints);
            robot_state->update();

            ROS_DEBUG("Updated joints: dq magnitude = %.6f", dq.norm());
        }

        // Max iterations reached
        ROS_WARN("JacobianIKSolver: Reached max iterations (%d) without convergence", max_iterations_);
        
        // Return best approximation found
        robot_state->copyJointGroupPositions(joint_model_group, joint_solution);
        return false;

    } catch (const std::exception& e) {
        ROS_ERROR("Exception in JacobianIKSolver::solveIK: %s", e.what());
        throw std::runtime_error(std::string("IK solving failed: ") + e.what());
    }
}

bool JacobianIKSolver::computeJacobian(const moveit::core::RobotStatePtr &robot_state,
                                       Eigen::MatrixXd &jacobian)
{
    try {
        const moveit::core::JointModelGroup* joint_model_group =
            robot_state->getJointModelGroup(planning_group_);

        if (!joint_model_group) {
            return false;
        }

        std::string ee_link = move_group_->getEndEffectorLink();
        if (ee_link.empty()) {
            ee_link = joint_model_group->getLinkModelNames().back();
        }

        // Get LinkModel for the end effector
        const moveit::core::LinkModel* ee_link_model = 
            robot_state->getLinkModel(ee_link);
        if (!ee_link_model) {
            ROS_ERROR("Failed to get link model for: %s", ee_link.c_str());
            return false;
        }

        // Compute Jacobian using MoveIt's built-in function
        Eigen::MatrixXd full_jacobian;
        bool success = robot_state->getJacobian(
            joint_model_group,
            ee_link_model,
            Eigen::Vector3d::Zero(),  // Reference point at link origin
            full_jacobian);

        if (!success) {
            ROS_ERROR("Failed to compute Jacobian");
            return false;
        }

        // Extract only position part (first 3 rows of Jacobian)
        // We only care about position error, not orientation for this simple version
        jacobian = full_jacobian.block(0, 0, 3, full_jacobian.cols());

        return true;

    } catch (const std::exception& e) {
        ROS_ERROR("Exception in computeJacobian: %s", e.what());
        return false;
    }
}

Eigen::Vector3d JacobianIKSolver::calculatePositionError(
    const Eigen::Vector3d &target,
    const Eigen::Vector3d &current)
{
    return target - current;
}

bool JacobianIKSolver::performJacobianStep(
    moveit::core::RobotStatePtr &robot_state,
    const geometry_msgs::Pose &target_pose)
{
    // This is a helper method - the main algorithm is in solveIK
    // Kept for future use or extensions
    return true;
}

// ===== PARAMETER SETTERS AND GETTERS =====

void JacobianIKSolver::setMaxIterations(int max_iter)
{
    max_iterations_ = max_iter;
    ROS_INFO("JacobianIKSolver: Max iterations set to %d", max_iterations_);
}

void JacobianIKSolver::setTolerance(double tol)
{
    tolerance_ = tol;
    ROS_INFO("JacobianIKSolver: Tolerance set to %.6f meters", tolerance_);
}

void JacobianIKSolver::setDampingFactor(double damping)
{
    damping_factor_ = damping;
    ROS_INFO("JacobianIKSolver: Damping factor set to %.6f", damping_factor_);
}

void JacobianIKSolver::setStepSize(double step)
{
    step_size_ = step;
    ROS_INFO("JacobianIKSolver: Step size set to %.6f", step_size_);
}

int JacobianIKSolver::getMaxIterations() const
{
    return max_iterations_;
}

double JacobianIKSolver::getTolerance() const
{
    return tolerance_;
}

double JacobianIKSolver::getDampingFactor() const
{
    return damping_factor_;
}

double JacobianIKSolver::getStepSize() const
{
    return step_size_;
}

} // namespace neura_motion_planning_challenge
