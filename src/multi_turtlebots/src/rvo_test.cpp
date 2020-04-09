#ifndef RVO_OUTPUT_TIME_AND_POSITIONS
#define RVO_OUTPUT_TIME_AND_POSITIONS 1
#endif


#if RVO_OUTPUT_TIME_AND_POSITIONS
#include <iostream>
#endif

#include <RVO.h>

std::vector<RVO::Vector2> goals;

void setupScenario(RVO::RVOSimulator* sim) {
  // Specify global time step of the simulation.
  sim->setTimeStep(0.25f);

  // setAgentDefaults (float neighborDist, size_t maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, const Vector2 &velocity=Vector2())
  // Specify default parameters for agents that are subsequently added.
  sim->setAgentDefaults(15.0f, 10, 10.0f, 5.0f, 2.0f, 2.0f);

  // Add agents, specifying their start position.
  sim->addAgent(RVO::Vector2(-50.0f, -50.0f));
  sim->addAgent(RVO::Vector2(50.0f, -50.0f));
  sim->addAgent(RVO::Vector2(50.0f, 50.0f));
  sim->addAgent(RVO::Vector2(-50.0f, 50.0f));

  // Create goals (simulator is unaware of these).
  for (size_t i = 0; i < sim->getNumAgents(); ++i) {
    goals.push_back(-sim->getAgentPosition(i));
  }

  // Add (polygonal) obstacle(s), specifying vertices in counterclockwise order.
  std::vector<RVO::Vector2> vertices;
  vertices.push_back(RVO::Vector2(-7.0f, -20.0f));
  vertices.push_back(RVO::Vector2(7.0f, -20.0f));
  vertices.push_back(RVO::Vector2(7.0f, 20.0f));
  vertices.push_back(RVO::Vector2(-7.0f, 20.0f));

  sim->addObstacle(vertices);

  // Process obstacles so that they are accounted for in the simulation.
  sim->processObstacles();
  // Obstacles added to the simulation after this function has been called are not accounted for in the simulatio
}

void setPreferredVelocities(RVO::RVOSimulator* sim) {
  // Set the preferred velocity for each agent.
  for (size_t i = 0; i < sim->getNumAgents(); ++i) {
    if (absSq(goals[i] - sim->getAgentPosition(i)) < sim->getAgentRadius(i) * sim->getAgentRadius(i) ) {
      // Agent is within one radius of its goal, set preferred velocity to zero
      sim->setAgentPrefVelocity(i, RVO::Vector2(0.0f, 0.0f));
    } else {
      // Agent is far away from its goal, set preferred velocity as unit vector towards agent's goal.
      sim->setAgentPrefVelocity(i, normalize(goals[i] - sim->getAgentPosition(i)));
    }
  }
}

void updateVisualization(RVO::RVOSimulator* sim) {
  // Output the current global time.
  std::cout << sim->getGlobalTime() << " ";

  // Output the position for all the agents.
  for (size_t i = 0; i < sim->getNumAgents(); ++i) {
    std::cout << sim->getAgentPosition(i) << " ";
  }

  std::cout << std::endl;
}

bool reachedGoal(RVO::RVOSimulator* sim) {
  // Check whether all agents have arrived at their goals.
  for (size_t i = 0; i < sim->getNumAgents(); ++i) {
    if (absSq(goals[i] - sim->getAgentPosition(i)) > sim->getAgentRadius(i) * sim->getAgentRadius(i)) {
      // Agent is further away from its goal than one radius.
      return false;
    }
  }
  return true;
}



int main()
{
  // Create a new simulator instance.
  RVO::RVOSimulator* sim = new RVO::RVOSimulator();

  // Set up the scenario.
  setupScenario(sim);

  // Perform (and manipulate) the simulation.
  do {
    updateVisualization(sim);
    setPreferredVelocities(sim);
    sim->doStep();
  } while (!reachedGoal(sim));

  delete sim;
}

