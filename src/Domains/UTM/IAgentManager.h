// Copyright 2016 Carrie Rebhuhn
#ifndef DOMAINS_UTM_UTMAGENTREWARD_H_
#define DOMAINS_UTM_UTMAGENTREWARD_H_

#include <string>
#include <memory>
#include <vector>

#include "Math/easymath.h"
#include "FileIO/FileOut.h"
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
    double alpha;
    size_t n_state_elements;

    virtual matrix2d compute_congestion_state(std::list<UAV*> &UAVs) = 0;
    //! Constructor that sets up agent management based on UTMModes.
    explicit IAgentManager(size_t n_agents, size_t set_n_state_elements);
    virtual ~IAgentManager() {}

    //! Identifies whether squared reward is used (D = G^2-G_c^2)
    bool square_reward;

    //! Global reward (not squared, even if square_reward set)
    matrix1d global();

    //! Replace the impact of the individual with the average delay
    matrix1d Gc_average();

    //! Remove the downstream traffic of an agent
    matrix1d Gc_downstream();

    //! Reallocate an individual's traffic to other agents
    matrix1d Gc_realloc();

    //! Remove any traffic that touches the individual during the run
    matrix1d Gc_touched();

    //! Zero counterfactual (G-Gc_0=G);
    matrix1d Gc_0();

    //! This is G-counterfactual (potentially squared)
    matrix1d reward();

    //! Global reward, squared if square_reward set
    matrix1d performance();

    //! Translates neural net output to link search costs
    virtual matrix2d actions2weights(matrix2d agent_actions) = 0;

    //! Stored agent actions, [*step][agent][action]
    matrix3d agentActions;

    //! Stored agent states, [*step][agent][state]
    matrix3d agentStates;

    //! Adds to agentActions
    void logAgentActions(matrix2d agentStepActions);

    //! Returns true if the last action was different.
    //! Used to prompt replanning.
    bool last_action_different();

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
            local(easymath::zeros(n_types)),
            G_avg(easymath::zeros(n_types)),
            G_minus_downstream(easymath::zeros(n_types)),
            G_random_realloc(easymath::zeros(n_types)),
            G_touched(easymath::zeros(n_types))
        {};

        matrix1d local;                 //! Local reward
        matrix1d G_avg;                 //! Average counterfactual
        matrix1d G_minus_downstream;    //! Downstream counterfactual
        matrix1d G_random_realloc;      //! Random reallocation counterfactual
        matrix1d G_touched;             //! Touched counterfactual
    };

    //! Resets for the next simulation call
    void reset();

    //! Stores metrics for each agent, used in reward calculation.
    std::vector<Reward_Metrics> metrics;
    //! Adds delay from UAV (based on agent definition)
    virtual void add_delay(UAV* u) = 0;

    //! Detects conflicts (based on agent definition)
    virtual void detect_conflicts() = 0;

    //! Simulates that step an agent replaced by its historical average.
    void add_average_counterfactual();

    //! Keeps track of Gc_downstream (based on agent definition)
    virtual void add_downstream_delay_counterfactual(UAV* u) = 0;
};
#endif  // DOMAINS_UTM_UTMAGENTREWARD_H_
