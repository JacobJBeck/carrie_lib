// Copyright 2016 Carrie Rebhuhn
#ifndef SRC_MULTIAGENT_INCLUDE_MULTIAGENTNE_H_
#define SRC_MULTIAGENT_INCLUDE_MULTIAGENTNE_H_

#include "IMultiagentSystem.h"
#include "Learning/include/NeuroEvo.h"
#include "Domains/IDomainStateful.h"

class MultiagentNE : public IMultiagentSystem<NeuroEvo> {
 public:
    MultiagentNE(void) {}
    MultiagentNE(size_t num_agents, size_t num_inputs, size_t num_hidden,
        size_t num_outputs);
    explicit MultiagentNE(IDomainStateful *domain);
    ~MultiagentNE(void);
    void generate_new_members();
    virtual void select_survivors();
    virtual bool set_next_pop_members();
};
#endif  // SRC_MULTIAGENT_INCLUDE_MULTIAGENTNE_H_
