#ifndef NEURA_MOTION_PLANNING_CHALLENGE_PLAN_COMPARATOR_H
#define NEURA_MOTION_PLANNING_CHALLENGE_PLAN_COMPARATOR_H

#include <vector>
#include <optional>
#include <neura_motion_planning_challenge/DataStructure/PlanMetaData.h>
#include <neura_motion_planning_challenge/DataStructure/PlanCriteria.h>

namespace neura_motion_planning_challenge
{

class PlanComparator {
public:
  static std::optional<PlanMetadata> getBestPlan(const std::vector<PlanMetadata>& plans, PlanEvaluationCriteria criteria);
  static std::vector<PlanMetadata> sortPlansByCriteria(const std::vector<PlanMetadata>& plans, PlanEvaluationCriteria criteria);
};

} // namespace neura_motion_planning_challenge

#endif // NEURA_MOTION_PLANNING_CHALLENGE_PLAN_COMPARATOR_H
