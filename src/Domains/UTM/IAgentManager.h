// Copyright 2016 Carrie Rebhuhn
#ifndef DOMAINS_UTM_UTMAGENTREWARD_H_
#define DOMAINS_UTM_UTMAGENTREWARD_H_

#include <string>
#include <memory>
#include <vector>

#include "Math/include/easymath.h"
#include "FileIO/include/FileOut.h"
#include "UAV.h"

/**
* Provides an interface for agents to interact with the simulator.
* This allows for redefinition of agents in the UTM simulation, and also
* collects information relevant to calculating difference, global, and
* local rewards. Logging of agent actions can also be performed for
* qualitative assessment of behavior.
*/

class IAgentManager {
 public:
    typedef matrix1d(IAgentManager::*counterfactual_method)();
    counterfactual_method counterfactual;
    double k_alpha_;
    size_t k_num_state_elements_;

    virtual matrix2d computeCongestionState(const std::list<UAV*> &UAVs) = 0;
    //! Constructor that sets up agent management based on UTMModes.
    explicit IAgentManager(size_t num_agents, size_t num_state_elements);
    virtual ~IAgentManager() {}

    //! Identifies whether squared reward is used (D = G^2-G_c^2)
    bool k_square_reward_mode_;

    //! Global reward (not squared, even if square_reward set)
    matrix1d global();

    //! Replace the impact of the individual with the average delay
    matrix1d GcAverage();

    //! Remove the downstream traffic of an agent
    matrix1d GcDownstream();

    //! Reallocate an individual's traffic to other agents
    matrix1d GcRealloc();

    //! Remove any traffic that touches the individual during the run
    matrix1d GcTouched();

    //! Zero counterfactual (G-Gc_0=G);
    matrix1d Gc0();

    //! This is G-counterfactual (potentially squared)
    matrix1d reward();

    //! Global reward, squared if square_reward set
    matrix1d performance();

    //! Translates neural net output to link search costs
    virtual matrix2d actionsToWeights(matrix2d agent_actions) = 0;

    //! Stored agent actions, [*step][agent][action]
    matrix3d agent_actions_;

    //! Stored agent states, [*step][agent][state]
    matrix3d agent_states_;

    //! Adds to agentActions
    void logAgentActions(matrix2d agentStepActions);

    //! Returns true if the last action was different.
    //! Used to prompt replanning.
    bool lastActionDifferent();

    //! Exports list of agent actions to a numbered file.
    void exportAgentActions(int fileID);


    struct Reward_Metrics {
        /**
        * Metrics relating to a reward.
        * This contains all information necessary to calculate an agent's
        * reward for a run. This is to collect all information in one place,
        * so that data for this calculation is not scattered all over the
        * simulator.
        */
        explicit Reward_Metrics(size_t n_types) :
            local_(easymath::zeros(n_types)),
            g_avg_(easymath::zeros(n_types)),
            g_minus_downstream_(easymath::zeros(n_types)),
            g_random_realloc_(easymath::zeros(n_types)),
            g_touched_(easymath::zeros(n_types))
        {};

        matrix1d local_;                 //! Local reward
        matrix1d g_avg_;                 //! Average counterfactual
        matrix1d g_minus_downstream_;    //! Downstream counterfactual
        matrix1d g_random_realloc_;      //! Random reallocation counterfactual
        matrix1d g_touched_;             //! Touched counterfactual
    };

    //! Resets for the next simulation call
    void reset();

    //! Stores metrics for each agent, used in reward calculation.
    std::vector<Reward_Metrics> metrics_;
    //! Adds delay from UAV (based on agent definition)
    virtual void addDelay(UAV* u) = 0;

    //! Detects conflicts (based on agent definition)
    virtual void detectConflicts() = 0;

    //! Simulates that step an agent replaced by its historical average.
    void addAverageCounterfactual();

    //! Keeps track of Gc_downstream (based on agent definition)
    virtual void addDownstreamDelayCounterfactual(UAV* u) = 0;
};
#endif  // DOMAINS_UTM_UTMAGENTREWARD_H_
