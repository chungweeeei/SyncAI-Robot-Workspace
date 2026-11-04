#ifndef SYNCAI_NAV_CORE__EXCEPTIONS_HPP_
#define SYNCAI_NAV_CORE__EXCEPTIONS_HPP_

#include <memory>
#include <stdexcept>
#include <string>

namespace syncai_nav_core
{

// What a controller throws for a cycle it cannot compute (the controller
// server tolerates it for failure_tolerance), and the base of every typed
// planning failure below.
class PlannerException : public std::runtime_error
{
public:
  explicit PlannerException(const std::string description) : std::runtime_error(description) {}
  using Ptr = std::shared_ptr<PlannerException>;
};

// Why a global planner has no path (2026-11): nav2 Iron's planner_exceptions
// under the same names, plus this stack's own. A planner throws one from
// createPlan() / createPlanThroughPoses() with its own sentence as what();
// planner_server catches it and answers the goal ABORTED with the matching
// syncai_common/ComputePathToPose error_code and the sentence as error_msg,
// which the navigator copies into NavigateToGoal / NavigateThroughGoals. An
// empty path is still a failure, read as PLAN_FAILED with no finer reason
// (the GlobalPlanner contract before 2026-11, kept for a planner that says
// nothing). The codes live in the interface, not here, so this package does
// not depend on syncai_common; the one mapping is planner_server's.
//
// Each class says whether the same goal sent again can succeed, which is
// what the backend decides a retry on (ComputePathToPose.action).
#define SYNCAI_NAV_CORE_PLANNER_EXCEPTION(Name)                                       \
  class Name : public PlannerException                                                \
  {                                                                                   \
  public:                                                                             \
    explicit Name(const std::string & description) : PlannerException(description) {} \
  };

// planner_id names no loaded planner (thrown by the server itself). Permanent.
SYNCAI_NAV_CORE_PLANNER_EXCEPTION(InvalidPlanner)
// No transform into the global frame, or no robot pose. May clear.
SYNCAI_NAV_CORE_PLANNER_EXCEPTION(PlannerTFError)
// Outside the map's grid. Permanent.
SYNCAI_NAV_CORE_PLANNER_EXCEPTION(StartOutsideMapBounds)
SYNCAI_NAV_CORE_PLANNER_EXCEPTION(GoalOutsideMapBounds)
// Nothing to stand on near it: a wall, a ledge, the wrong floor, a person.
// May clear.
SYNCAI_NAV_CORE_PLANNER_EXCEPTION(StartOccupied)
SYNCAI_NAV_CORE_PLANNER_EXCEPTION(GoalOccupied)
// Out of time or of iterations. May clear.
SYNCAI_NAV_CORE_PLANNER_EXCEPTION(PlannerTimedOut)
// The search exhausted the reachable ground. May clear (a person) or not (a
// zone); the sentence says which are present.
SYNCAI_NAV_CORE_PLANNER_EXCEPTION(NoValidPathCouldBeFound)
// A route with no waypoints. Permanent.
SYNCAI_NAV_CORE_PLANNER_EXCEPTION(NoViapointsGiven)
// This stack's own (no nav2 counterpart): no map loaded (permanent until a
// map switch), the map still being built (clears), the robot inside a
// keepout zone (until it is moved), the goal inside one (until the zone is
// edited), start and goal on ground no stair joins (permanent).
SYNCAI_NAV_CORE_PLANNER_EXCEPTION(MapNotLoaded)
SYNCAI_NAV_CORE_PLANNER_EXCEPTION(MapLoading)
SYNCAI_NAV_CORE_PLANNER_EXCEPTION(StartInKeepout)
SYNCAI_NAV_CORE_PLANNER_EXCEPTION(GoalInKeepout)
SYNCAI_NAV_CORE_PLANNER_EXCEPTION(GroundDisconnected)

#undef SYNCAI_NAV_CORE_PLANNER_EXCEPTION

}  // namespace syncai_nav_core

#endif  // SYNCAI_NAV_CORE__EXCEPTIONS_HPP_
