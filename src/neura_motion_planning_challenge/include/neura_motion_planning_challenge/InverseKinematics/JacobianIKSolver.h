#ifndef NEURA_MOTION_PLANNING_CHALLENGE_JACOBIAN_IK_SOLVER_H
#define NEURA_MOTION_PLANNING_CHALLENGE_JACOBIAN_IK_SOLVER_H

#include <neura_motion_planning_challenge/InverseKinematics/IKStrategy.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <memory>
#include <Eigen/Dense>

namespace neura_motion_planning_challenge
{

/**
 * @brief Jacobian-based inverse kinematics solver using numerical methods.
 *
 * This solver implements iterative IK solving using the Jacobian matrix and
 * damped least squares (pseudo-inverse) method. It is excellent for solving
 * waypoints in Cartesian space planning because:
 *
 * 1. **Fast Convergence**: Iteratively refines the solution quickly
 * 2. **Numerical Stability**: Uses damped pseudo-inverse to avoid singularities
 * 3. **General Purpose**: Works with any robot without analytical solutions
 * 4. **Controllable**: Tune convergence speed with damping and step size
 * 5. **Independent**: Doesn't rely on pre-configured IK solvers
 *
 * Algorithm:
 * - Computes Jacobian matrix (relationship between joint velocities and end-effector velocity)
 * - Uses damped least squares: Δq = (J^T * J + λI)^-1 * J^T * Δx
 * - Iteratively updates joint configuration until error is minimized
 * - Stops when position error or iteration limit is reached
 *
 * @note This is a numerical approach that requires iteration and may find
 *       local minima. Good for smooth, nearby trajectories (like your waypoints).
 */
class JacobianIKSolver : public IKStrategy
{
public:
    /**
     * @brief Constructor for JacobianIKSolver.
     *
     * @param move_group Shared pointer to MoveGroupInterface.
     *                  Used to access robot model and joint information.
     * @param planning_group The name of the planning group (e.g., "arm").
     * @param max_iterations Maximum number of IK iterations. Default: 100.
     *                       More iterations increase accuracy but take longer.
     * @param tolerance Position error tolerance in meters. Default: 0.001 (1mm).
     *                 Smaller values are more precise but may not converge.
     * @param damping_factor Damping coefficient λ for numerical stability. Default: 0.01.
     *                      Higher values avoid singularities but reduce accuracy.
     *                      Range: 0.001 to 0.1
     * @param step_size Scaling factor for joint updates. Default: 1.0.
     *                 Smaller values are more stable but converge slower.
     *
     * @throws std::runtime_error If move_group is null.
     */
    JacobianIKSolver(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
                     const std::string &planning_group = "arm",
                     int max_iterations = 100,
                     double tolerance = 0.001,
                     double damping_factor = 0.01,
                     double step_size = 1.0);

    /**
     * @brief Destructor.
     */
    ~JacobianIKSolver() override = default;

    /**
     * @brief Solves inverse kinematics using Jacobian-based numerical method.
     *
     * This method uses an iterative algorithm:
     * 1. Start from current robot state
     * 2. Compute Jacobian matrix at current joint configuration
     * 3. Calculate position error (target - current end-effector)
     * 4. Update joints using damped least squares: Δq = (J^T*J + λI)^-1 * J^T * Δx
     * 5. Repeat until error < tolerance or max iterations reached
     *
     * @param target_pose The desired end-effector pose.
     * @param joint_solution Output: the solved joint configuration (in radians).
     *
     * @return true if convergence achieved (error < tolerance), false otherwise.
     *
     * @throws std::runtime_error If an error occurs during IK computation.
     *
     * @note If convergence is not achieved but we're close, some implementations
     *       may return true with best approximation. Check tolerance parameter.
     */
    bool solveIK(const geometry_msgs::Pose &target_pose,
                std::vector<double> &joint_solution) override;

    /**
     * @brief Set the maximum number of iterations.
     *
     * @param max_iter Number of iterations. Default: 100.
     *                Higher values increase chances of convergence.
     */
    void setMaxIterations(int max_iter);

    /**
     * @brief Set the position error tolerance.
     *
     * @param tol Tolerance in meters. Default: 0.001 (1mm).
     *           Smaller values require more accuracy (harder to achieve).
     */
    void setTolerance(double tol);

    /**
     * @brief Set the damping factor for numerical stability.
     *
     * @param damping Damping coefficient λ. Default: 0.01.
     *               Range: 0.001 (less stable) to 0.1 (more stable).
     *               Higher values help near singularities but reduce precision.
     */
    void setDampingFactor(double damping);

    /**
     * @brief Set the step size for joint updates.
     *
     * @param step Step size multiplier. Default: 1.0.
     *            Range: 0.1 (slow) to 1.0 (fast).
     *            Smaller values increase stability.
     */
    void setStepSize(double step);

    /**
     * @brief Get current parameter values.
     */
    int getMaxIterations() const;
    double getTolerance() const;
    double getDampingFactor() const;
    double getStepSize() const;

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::string planning_group_;
    int max_iterations_;
    double tolerance_;
    double damping_factor_;
    double step_size_;

    /**
     * @brief Computes the Jacobian matrix for current joint configuration.
     *
     * @param robot_state Current state of the robot.
     * @param jacobian Output: the computed Jacobian matrix.
     * @return true if successful.
     */
    bool computeJacobian(const moveit::core::RobotStatePtr &robot_state,
                        Eigen::MatrixXd &jacobian);

    /**
     * @brief Calculates position error between target and current end-effector.
     *
     * @param target Target Cartesian position (x, y, z).
     * @param current Current end-effector position.
     * @return Position error vector (3D).
     */
    Eigen::Vector3d calculatePositionError(const Eigen::Vector3d &target,
                                          const Eigen::Vector3d &current);

    /**
     * @brief Performs one iteration of Jacobian IK refinement.
     *
     * @param robot_state Robot state to update (modified in-place).
     * @param target_pose Target end-effector pose.
     * @return true if iteration was successful.
     */
    bool performJacobianStep(moveit::core::RobotStatePtr &robot_state,
                            const geometry_msgs::Pose &target_pose);
};

} // namespace neura_motion_planning_challenge

#endif // NEURA_MOTION_PLANNING_CHALLENGE_JACOBIAN_IK_SOLVER_H
