#ifndef NEURA_MOTION_PLANNING_CHALLENGE_IK_STRATEGY_H
#define NEURA_MOTION_PLANNING_CHALLENGE_IK_STRATEGY_H

#include <geometry_msgs/Pose.h>
#include <vector>

namespace neura_motion_planning_challenge
{

/**
 * @brief Abstract base class for Inverse Kinematics solvers.
 *
 * Defines the interface that all IK solving algorithms must implement.
 * Different IK algorithms (MoveIt, Jacobian, Analytical, etc.) can be
 * plugged in by implementing this interface.
 *
 * This allows switching between different IK solvers at runtime without
 * modifying the motion planning code.
 */
class IKStrategy
{
public:
    /**
     * @brief Pure virtual function to solve inverse kinematics.
     *
     * Finds joint configurations that achieve a target Cartesian pose.
     *
     * @param target_pose The desired Cartesian pose of the end-effector.
     * @param joint_solution Output: the solved joint configuration (in radians).
     *                      Must contain values for all joints in the planning group.
     *
     * @return true if an IK solution was found, false otherwise.
     *
     * @throws std::runtime_error If an error occurs during IK solving.
     *
     * @note If multiple solutions exist, the solver may return any valid solution.
     *       The specific solution chosen depends on the algorithm implementation.
     */
    virtual bool solveIK(const geometry_msgs::Pose &target_pose,
                        std::vector<double> &joint_solution) = 0;

    /**
     * @brief Virtual destructor for proper cleanup of derived classes.
     */
    virtual ~IKStrategy() = default;

protected:
    /**
     * @brief Protected constructor.
     *
     * Protected because this is an abstract class and cannot be instantiated directly.
     */
    IKStrategy() = default;

private:
    // Prevent copying of polymorphic classes
    IKStrategy(const IKStrategy&) = delete;
    IKStrategy& operator=(const IKStrategy&) = delete;
};

} // namespace neura_motion_planning_challenge

#endif // NEURA_MOTION_PLANNING_CHALLENGE_IK_STRATEGY_H
