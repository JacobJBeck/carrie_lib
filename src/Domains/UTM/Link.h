// Copyright 2016 Carrie Rebhuhn
#ifndef DOMAINS_UTM_LINK_H_
#define DOMAINS_UTM_LINK_H_

#include <list>
#include <vector>

#include "IAgentManager.h"

class Link {
 public:
     Link(size_t ID, int source_set, int target_set, int time,
         std::vector<size_t> capacity, int cardinal_dir, size_t n_types_set);

    //!
     bool at_capacity(size_t UAV_type);

     int number_over_capacity(size_t type_ID);
    std::vector<std::list<UAV*> > traffic;
    size_t count_traffic() {
        size_t count = 0;
        for (std::list<UAV*> t : traffic) {
            count += t.size();
        }
        return count;
    }


    //! Returns the predicted amount of time it would take to cross the node if
    //! the UAV got there immediately
    matrix1d predicted_traversal_time();

    //! Grabs the UAV u from link l
    void move_from(UAV* u, Link* l);

    //! Also sets the time
    void add(UAV* u);

    void remove(UAV* u);

    const int source;
    const int target;
    const int cardinal_dir;
    void reset();


 private:
    const size_t ID;
    const int time;  // Amount of time it takes to travel across link
    size_t n_types;
    std::vector<size_t> capacity;  // Capacity for each UAV type [#types]
};

/**
* Provides an interface for link agents to interact with the simulator.
* This allows for redefinition of agents in the UTM simulation, and also
* collects information relevant to calculating difference, global, and
* local rewards. Logging of agent actions can also be performed for
* qualitative assessment of behavior.
*/

class LinkAgentManager : public IAgentManager {
 public:
    // The agent that communicates with others
    LinkAgentManager(int n_edges, int n_types, std::vector<Link*> links,
        size_t set_n_state_elements);
    virtual ~LinkAgentManager() {}
    // weights are ntypesxnagents

    const int n_edges;
    const int n_types;

    /**
    * Translates the output of a neural network into costs applied to a link.
    * This can include the 'predicted' cost of the link, which is the
    * traversal time plus the instantaneous wait time at that link. In
    * addition, this translates neural network output, which is in the form
    * [agent #][type #] into weights on the graph, which is in the form
    * [type #][link #]. In the case of link agents, this mapping is
    * agent # = link #, but this is not the case with sector agents.
    * @param agent_actions neural network output, in the form of [agent #][type #]
    * @return the costs for each link in the graph
    */
    virtual matrix2d actions2weights(matrix2d agent_actions);

    std::vector<Link*> links;

    /**
    * Adds to the delay for the agent assigned to that link.
    * Agent reward metrics and link function are kept separate.
    */
    virtual void add_delay(UAV* u);
    virtual matrix2d compute_congestion_state(std::list<UAV*>& UAVs) {
        size_t n_agents = links.size();
        matrix2d allStates = easymath::zeros(n_agents, n_state_elements);
        
        for (UAV* u : UAVs)
            allStates[u->get_cur_link()][u->get_type()]++;
        agentStates.push_back(allStates);
        return allStates;
    }
    void add_downstream_delay_counterfactual(UAV* u);

    void detect_conflicts();
};
#endif  // DOMAINS_UTM_LINK_H_
