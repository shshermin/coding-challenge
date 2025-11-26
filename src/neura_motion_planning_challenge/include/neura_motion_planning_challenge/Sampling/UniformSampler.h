#ifndef NEURA_MOTION_PLANNING_CHALLENGE_UNIFORM_SAMPLER_H
#define NEURA_MOTION_PLANNING_CHALLENGE_UNIFORM_SAMPLER_H

#include <neura_motion_planning_challenge/Sampling/SamplingStrategy.h>

namespace neura_motion_planning_challenge
{

/**
 * @brief Concrete implementation of uniform sampling strategy.
 *
 * Generates waypoints at equal intervals along the Cartesian path.
 * Uses linear interpolation for position and SLERP (Spherical Linear Interpolation)
 * for orientation to ensure smooth rotation.
 *
 * This is the simplest and most commonly used sampling strategy for direct
 * Cartesian paths (e.g., straight line from point A to point B).
 */
class UniformSampler : public SamplingStrategy
{
public:
    /**
     * @brief Constructor for UniformSampler.
     */
    UniformSampler() = default;

    /**
     * @brief Destructor for UniformSampler.
     */
    ~UniformSampler() override = default;

    /**
     * @brief Generate uniformly spaced waypoints between two poses.
     *
     * Creates num_waypoints evenly distributed along the Cartesian path from
     * start to end. Each waypoint is generated at parameter t = i / (num_waypoints - 1),
     * where i ranges from 0 to num_waypoints - 1.
     *
     * Position interpolation: Linear interpolation in 3D space
     * Orientation interpolation: SLERP for smooth rotation
     *
     * @param start The starting Cartesian waypoint (point A).
     * @param end The ending Cartesian waypoint (point B).
     * @param num_waypoints The number of waypoints to generate (including start and end).
     *                      Must be at least 2.
     *
     * @return A vector of CartMotionPlanningData representing the sampled waypoints.
     *
     * @throws std::runtime_error If num_waypoints < 2.
     */
    std::vector<CartMotionPlanningData> generateWaypoints(
        const CartMotionPlanningData &start,
        const CartMotionPlanningData &end,
        int num_waypoints) override;
};

} // namespace neura_motion_planning_challenge

#endif // NEURA_MOTION_PLANNING_CHALLENGE_UNIFORM_SAMPLER_H
