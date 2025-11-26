#ifndef NEURA_MOTION_PLANNING_CHALLENGE_SAMPLING_STRATEGY_H
#define NEURA_MOTION_PLANNING_CHALLENGE_SAMPLING_STRATEGY_H

#include <neura_motion_planning_challenge/DataStructure/CartMotionPlanningData.h>
#include <vector>

namespace neura_motion_planning_challenge
{

/**
 * @brief Abstract base class for sampling strategies.
 *
 * Defines the interface for generating waypoints along a Cartesian path.
 * Derived classes must implement the generateWaypoints method.
 *
 * This class uses pure virtual functions to enforce implementation
 * in derived classes and prevent instantiation of the base class.
 */
class SamplingStrategy
{
public:
    /**
     * @brief Virtual destructor - required for polymorphic classes.
     *
     * Ensures proper cleanup of derived class resources.
     */
    virtual ~SamplingStrategy() = default;

    /**
     * @brief Pure virtual function to generate waypoints between two poses.
     *
     * Derived classes MUST implement this method.
     *
     * @param start The starting Cartesian waypoint (point A).
     * @param end The ending Cartesian waypoint (point B).
     * @param num_waypoints The number of waypoints to generate (including start and end).
     *
     * @return A vector of CartMotionPlanningData representing the sampled waypoints.
     *
     * @throws std::runtime_error If waypoint generation fails.
     */
    virtual std::vector<CartMotionPlanningData> generateWaypoints(
        const CartMotionPlanningData &start,
        const CartMotionPlanningData &end,
        int num_waypoints) = 0;

protected:
    /**
     * @brief Protected constructor.
     *
     * Protected constructor allows derived classes to use it.
     * Abstract class cannot be instantiated directly.
     */
    SamplingStrategy() = default;

private:
    // Prevent copying 
    SamplingStrategy(const SamplingStrategy&) = delete;
    SamplingStrategy& operator=(const SamplingStrategy&) = delete;
};

} // namespace neura_motion_planning_challenge

#endif // NEURA_MOTION_PLANNING_CHALLENGE_SAMPLING_STRATEGY_H
