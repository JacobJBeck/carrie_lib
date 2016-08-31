// Copyright 2016 Carrie Rebhuhn
#ifndef MULTIAGENT_IMULTIAGENTSYSTEM_H_
#define MULTIAGENT_IMULTIAGENTSYSTEM_H_


// STL includes
#include <vector>

// Library includes
#include "Learning/IAgent.h"

//! Agents container
template<class Agent>
class IMultiagentSystem {
 public:
     typedef typename Agent::State State;
     typedef typename Agent::Action Action;
     typedef typename Agent::Reward Reward;

     IMultiagentSystem(void) {};
     virtual ~IMultiagentSystem(void) {};

    // Set of agents in the system (set externally)
    std::vector<Agent*> agents;

    std::vector<Action> get_actions(std::vector<State> S) {
        std::vector<Action> A(S.size());
        // get all actions, given a list of states
        for (size_t i = 0; i < agents.size(); i++) {
            A[i] = agents[i]->get_action(S[i]);
        }
        return A;
    }
    inline void update_policy_values(std::vector<Reward> R) {
        for (size_t i = 0; i < agents.size(); i++)
            agents[i]->update_policy_values(R[i]);
    }
};
#endif  // MULTIAGENT_IMULTIAGENTSYSTEM_H_
