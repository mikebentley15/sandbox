// This one is using ompl::geometric::SimpleSetup
// Tutorial from
//   http://ompl.kavrakilab.org/geometricPlanningSE3.html

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <iostream>
#include <memory>

#define UNUSED_VAR(x) (void)x

namespace ob = ompl::base;
namespace og = ompl::geometric;

class ValidityChecker : public ob::StateValidityChecker {
public:
  ValidityChecker(const ob::SpaceInformationPtr &si) :
    ob::StateValidityChecker(si) {}

  virtual bool isValid(const ob::State *state) const {
    const auto *se3state = state->as<ob::SE3StateSpace::StateType>();
    const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
    const auto &rot = se3state->rotation();
    UNUSED_VAR(pos);
    UNUSED_VAR(rot);
    // TODO: collision detection
    return true;
  }
};

// Could use this instead of the ValidityChecker class
bool isStateValid(const ob::State *state) {
  const auto *se3state = state->as<ob::SE3StateSpace::StateType>();
  const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
  const auto &rot = se3state->rotation();
  UNUSED_VAR(pos);
  UNUSED_VAR(rot);
  // TODO: collision detection
  return true;
}

void planWithoutSimpleSetup() {
  // construct the state space for planning
  auto space(std::make_shared<ob::SE3StateSpace>());

  // set the bounds of the state space
  ob::RealVectorBounds bounds(3);
  bounds.setLow(-1);
  bounds.setHigh(1);

  space->setBounds(bounds);

  // create the space information
  auto si(std::make_shared<ob::SpaceInformation>(space));
  si->setStateValidityChecker(std::make_shared<ValidityChecker>(si));

  // random start and goal
  ob::ScopedState<> start(space);
  ob::ScopedState<> goal(space);
  start.random();
  goal.random();

  // create the problem definition
  auto pdef(std::make_shared<ob::ProblemDefinition>(si));
  pdef->setStartAndGoalStates(start, goal);

  // plan
  auto planner(std::make_shared<og::RRTConnect>(si));
  planner->setProblemDefinition(pdef);
  planner->setup();

  // solve
  double timeout = 1.0;
  auto solve_status = planner->ob::Planner::solve(timeout);

  if (solve_status) {
    // get the goal representation from the problem definition (not the same as
    // the goal state)
    // inquire about the found path
    auto path = pdef->getSolutionPath();
    std::cout << "Found solution:" << std::endl;

    // print the path
    path->print(std::cout);
  } else {
    std::cout << "No solution found" << std::endl;
  }
}

void planWithSimpleSetup() {
  // construct the state space for planning
  auto space(std::make_shared<ob::SE3StateSpace>());

  // Set the bounds of the state space
  ob::RealVectorBounds bounds(3);
  bounds.setLow(-1);
  bounds.setHigh(1);

  space->setBounds(bounds);

  // Create an instance of og::SimpleSetup.
  // Note: ob::SpaceInformation and ob::ProblemDefinition are created
  //       internally
  og::SimpleSetup ss(space);

  // Set the state validity checker
  ss.setStateValidityChecker([](const ob::State *state) {
    return isStateValid(state);
  });

  // Create a random start state
  ob::ScopedState<> start(space);
  start.random();

  // Create a random goal state
  ob::ScopedState<> goal(space);
  goal.random();

  // Set these as the start and goals of the setup
  ss.setStartAndGoalStates(start, goal);

  // Note: lots of magic is going on within the SimpleSetup.  See the tutorial
  // webpage for more information.

  // Solve for a plan
  ob::PlannerStatus solved = ss.solve(1.0);

  if (solved) {
    std::cout << "Found solution:" << std::endl;
    // print the path to screen
    ss.simplifySolution();
    ss.getSolutionPath().print(std::cout);
  } else {
    std::cout << "No solution found" << std::endl;
  }
}

int main(int argCount, char *argList[]) {
  UNUSED_VAR(argCount);
  UNUSED_VAR(argList);

  std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

  planWithoutSimpleSetup();

  std::cout << std::endl << std::endl;

  planWithSimpleSetup();

  return 0;
}
