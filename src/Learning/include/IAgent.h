// Copyright 2016 Carrie Rebhuhn
#ifndef SINGLEAGENT_IAGENT_H_
#define SINGLEAGENT_IAGENT_H_

#include "IPolicy.h"

template<class Policy>
class IAgent {
 public:
    typedef typename Policy::State State;
    typedef typename Policy::Action Action;
    typedef typename Policy::Reward Reward;

    //! Life cycle
    IAgent(void) {}
    virtual ~IAgent(void) {}

    //! Accessors
    Action get_action(State state) { return (*policy)(state); }

    //! Mutators
    void update_policy_values(Reward R) { policy->update(R); }
    void set_policy(Policy* p) { policy = p; }

 private:
    Policy* policy;
};

#endif  // SINGLEAGENT_IAGENT_H_
