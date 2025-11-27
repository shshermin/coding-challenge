#include <neura_motion_planning_challenge/Utility/PlanComparator.h>
#include <algorithm>
#include <stdexcept>

using namespace neura_motion_planning_challenge;

/**
 * @brief Finds the best plan from a collection of plans based on a criteria.
 * 
 * Compares all plans and returns the one with the lowest value for the specified criteria.
 * 
 * @param plans The vector of PlanMetadata objects to evaluate.
 * @param criteria The evaluation criteria (LENGTH, SUM_ABS_JOINTS, or PLAN_TIME).
 * @return An optional containing the best plan, or std::nullopt if plans is empty.
 */
std::optional<PlanMetadata> PlanComparator::getBestPlan(const std::vector<PlanMetadata>& plans, PlanEvaluationCriteria criteria) {
    if (plans.empty()) {
        return std::nullopt;
    }
    return *std::min_element(plans.begin(), plans.end(),
        [criteria](const PlanMetadata& a, const PlanMetadata& b) {
            return a.getCriteriaValue(criteria) < b.getCriteriaValue(criteria);
        });
}

/**
 * @brief Sorts a collection of plans by a specified evaluation criteria.
 * 
 * Creates a sorted copy of the input plans, ordered from lowest to highest value
 * for the specified criteria.
 * 
 * @param plans The vector of PlanMetadata objects to sort.
 * @param criteria The evaluation criteria (LENGTH, SUM_ABS_JOINTS, or PLAN_TIME).
 * @return A sorted vector of PlanMetadata objects.
 */
std::vector<PlanMetadata> PlanComparator::sortPlansByCriteria(const std::vector<PlanMetadata>& plans, PlanEvaluationCriteria criteria) {
    std::vector<PlanMetadata> sorted = plans;
    std::sort(sorted.begin(), sorted.end(),
        [criteria](const PlanMetadata& a, const PlanMetadata& b) {
            return a.getCriteriaValue(criteria) < b.getCriteriaValue(criteria);
        });
    return sorted;
}
