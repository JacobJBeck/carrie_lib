// Copyright 2016 Carrie Rebhuhn
#include "MultiagentNE.h"
#include <vector>
#include <stdio.h>

using std::vector;

MultiagentNE::MultiagentNE(size_t n_agents, NeuroEvoParameters* NE_params) :
    NE_params(NE_params) {
    std::printf("Creating a MultiagentNE object.\n");
    for (int i = 0; i < n_agents; i++) {
        agents.push_back(new NeuroEvo(NE_params));
    }
}

MultiagentNE::MultiagentNE(IDomainStateful* domain):
    MultiagentNE(domain->get_n_agents(),
        new NeuroEvoParameters(domain->get_nn_inputs(),
            domain->get_nn_outputs()))
{}

MultiagentNE::~MultiagentNE(void) {
    for (size_t i = 0; i < agents.size(); i++) {
        delete agents[i];
    }
}

void MultiagentNE::generate_new_members() {
    // Generate new population members
    for (size_t i = 0; i < agents.size(); i++) {
        reinterpret_cast<NeuroEvo*>(agents[i])->generate_new_members();
    }
}

void MultiagentNE::select_survivors() {
    // Specific to Evo: select survivors
    for (size_t i = 0; i < agents.size(); i++) {
        reinterpret_cast<NeuroEvo*>(agents[i])->select_survivors();
    }
}

bool MultiagentNE::set_next_pop_members() {
    // Kind of hacky; select the next member and return true if not at the end
    // Specific to Evo

    vector<bool> is_another_member(agents.size(), false);
    for (size_t i = 0; i < agents.size(); i++) {
        is_another_member[i]
            = reinterpret_cast<NeuroEvo*>(agents[i])->select_new_member();
    }
    for (size_t i = 0; i < is_another_member.size(); i++) {
        if (!is_another_member[i]) {
            return false;
        }
    }
    return true;
}
