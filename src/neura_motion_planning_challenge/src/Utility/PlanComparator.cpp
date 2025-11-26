#include <neura_motion_planning_challenge/Utility/PlanComparator.h>
#include <algorithm>
#include <stdexcept>

using namespace neura_motion_planning_challenge;

std::optional<PlanMetadata> PlanComparator::getBestPlan(const std::vector<PlanMetadata>& plans, PlanCriteria criteria) {
    if (plans.empty()) {
        return std::nullopt;
    }
    return *std::min_element(plans.begin(), plans.end(),
        [criteria](const PlanMetadata& a, const PlanMetadata& b) {
            return a.getCriteriaValue(criteria) < b.getCriteriaValue(criteria);
        });
}

std::vector<PlanMetadata> PlanComparator::sortPlansByCriteria(const std::vector<PlanMetadata>& plans, PlanCriteria criteria) {
    std::vector<PlanMetadata> sorted = plans;
    std::sort(sorted.begin(), sorted.end(),
        [criteria](const PlanMetadata& a, const PlanMetadata& b) {
            return a.getCriteriaValue(criteria) < b.getCriteriaValue(criteria);
        });
    return sorted;
}
