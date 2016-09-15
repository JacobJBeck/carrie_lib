// Copyright 2016 Carrie Rebhuhn
#ifndef SRC_DOMAINS_UTM_LINK_H_
#define SRC_DOMAINS_UTM_LINK_H_

#include <list>
#include <vector>

#include "IAgentManager.h"

class Link {
 public:
     enum {NOT_ON_LINK};  // link exceptions
     Link(size_t id, size_t source, size_t target, size_t time,
         std::vector<size_t> capacity, size_t cardinal_dir, size_t num_types);

    //!
     bool atCapacity(size_t UAV_type);

     int numOverCapacity(size_t type_ID);
    std::vector<std::list<UAV*> > traffic_;
    size_t countTraffic() {
        size_t count = 0;
        for (std::list<UAV*> t : traffic_) {
            count += t.size();
        }
        return count;
    }


    //! Returns the predicted amount of time it would take to cross the node if
    //! the UAV got there immediately
    matrix1d predictedTraversalTime();

    //! Grabs the UAV u from link l
    void moveFrom(UAV* u, Link* l);

    //! Also sets the time
    void add(UAV* u);

    void remove(UAV* u);

    const int k_source_;
    const int k_target_;
    const int k_cardinal_dir_;
    void reset();


 private:
    const size_t k_id_;
    const int time_;  // Amount of time it takes to travel across link
    size_t k_num_types_;
    std::vector<size_t> k_capacity_;  // Capacity for each UAV type [#types]
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
    LinkAgentManager(size_t num_edges, size_t num_types,
        std::vector<Link*> links, size_t num_state_elements);
    virtual ~LinkAgentManager() {}
    // weights are ntypesxnagents

    const size_t k_num_edges_, k_num_types_;

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
    virtual matrix2d actionsToWeights(matrix2d agent_actions);

    std::vector<Link*> links_;
    std::map<std::pair<size_t, size_t>, size_t> k_link_ids_;

    size_t getNthLink(UAV* u, size_t n) {
        return k_link_ids_[u->getNthEdge(n)];
    }

    /**
    * Adds to the delay for the agent assigned to that link.
    * Agent reward metrics and link function are kept separate.
    */
    virtual void addDelay(UAV* u);
    matrix2d computeCongestionState(const std::list<UAV*>& uavs) {
        size_t num_agents = links_.size();
        matrix2d all_states = easymath::zeros(num_agents,
            k_num_state_elements_);
        for (UAV* u : uavs)
            all_states[getNthLink(u,0)][STATIC_TYPE]++;
        agent_states_.push_back(all_states);
        return all_states;
    }
    void addDownstreamDelayCounterfactual(UAV* u);

    void detectConflicts();
};
#endif  // SRC_DOMAINS_UTM_LINK_H_
