// Copyright 2016 Carrie Rebhuhn
#ifndef MULTIAGENT_MULTIAGENTNE_H_
#define MULTIAGENT_MULTIAGENTNE_H_

#include "IMultiagentSystem.h"
#include "Learning/include/NeuroEvo.h"
#include "Domains/IDomainStateful.h"

class MultiagentNE : public IMultiagentSystem<NeuroEvo> {
 public:
    MultiagentNE(void) {}
    MultiagentNE(size_t n_agents, NeuroEvoParameters* NE_params);
    MultiagentNE(IDomainStateful *domain);
    ~MultiagentNE(void);
    void generate_new_members();
    virtual void select_survivors();
    virtual bool set_next_pop_members();
    NeuroEvoParameters* NE_params;
};
#endif  // MULTIAGENT_MULTIAGENTNE_H_
