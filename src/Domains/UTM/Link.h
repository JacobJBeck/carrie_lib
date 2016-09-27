// Copyright 2016 Carrie Rebhuhn
#ifndef SRC_DOMAINS_UTM_LINK_H_
#define SRC_DOMAINS_UTM_LINK_H_

#include <list>
#include <vector>

#include "IAgentBody.h"

class Link {
 public:
     enum {NOT_ON_LINK};  // link exceptions
     Link(size_t id, size_t source, size_t target, size_t time,
         size_t capacity, size_t cardinal_dir);

    //!
     bool atCapacity();

     int numOverCapacity();
    std::list<UAV*> traffic_;
    size_t countTraffic() {
        return traffic_.size();
    }


    //! Returns the predicted amount of time it would take to cross the node if
    //! the UAV got there immediately
    double predictedTraversalTime();

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
    size_t k_capacity_;  // Capacity for each UAV type [#types]
};

/**
* Provides an interface for link agents to interact with the simulator.
* This allows for redefinition of agents in the UTM simulation, and also
* collects information relevant to calculating difference, global, and
* local rewards. Logging of agent actions can also be performed for
* qualitative assessment of behavior.
*/

class LinkAgent : public IAgentBody {
 public:
    // The agent that communicates with others
    LinkAgent(size_t num_edges,
        std::vector<Link*> links, size_t num_state_elements);
    virtual ~LinkAgent() {}
    // weights are ntypesxnagents

    const size_t k_num_edges_;

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
    virtual matrix1d actionsToWeights(matrix2d agent_actions);

    std::vector<Link*> links_;
    std::map<std::pair<size_t, size_t>, size_t> k_link_ids_;

    size_t getNthLink(UAV* u, size_t n) {
        return k_link_ids_[u->getNthEdge(n)];
    }

    matrix2d computeCongestionState(const std::list<UAV*>& uavs) {
        size_t num_agents = links_.size();
        matrix2d all_states = easymath::zeros(num_agents,
            1);
        for (UAV* u : uavs)
            all_states[getNthLink(u,0)][0]++;
        agent_states_.push_back(all_states);
        return all_states;
    }
};
#endif  // SRC_DOMAINS_UTM_LINK_H_
