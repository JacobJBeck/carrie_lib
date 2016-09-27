// Copyright 2016 Carrie Rebhuhn
#ifndef DOMAINS_UTM_UTMAGENTREWARD_H_
#define DOMAINS_UTM_UTMAGENTREWARD_H_

#include <string>
#include <memory>
#include <vector>

#include "Math/include/easymath.h"
#include "FileIO/include/FileOut.h"
#include "UAV.h"

class IAgentBody {
public:
    // Life cycle
    explicit IAgentBody(size_t num_agents, size_t num_states);
    virtual ~IAgentBody() {}
    //! Resets for the next simulation call
    void reset();

    
    // State
    virtual matrix2d computeCongestionState(const std::list<UAV*> &UAVs) = 0;
    //! Stored agent states, [*step][agent][state]
    matrix3d agent_states_;
    size_t k_num_states_;

    // Actions
    //! Translates neural net output to link search costs
    virtual matrix1d actionsToWeights(matrix2d agent_actions) = 0;
    //! Stored agent actions, [*step][agent][action]
    matrix3d agent_actions_;
    //! Adds to agentActions
    void logAgentActions(matrix2d agentStepActions);
    //! Returns true if the last action was different.
    //! Used to prompt replanning.
    bool lastActionDifferent();
    //! Exports list of agent actions to a numbered file.
    void exportAgentActions(int fileID);


    // Reward
    double k_alpha_;
    bool k_square_reward_mode_;
};
#endif  // DOMAINS_UTM_UTMAGENTREWARD_H_
