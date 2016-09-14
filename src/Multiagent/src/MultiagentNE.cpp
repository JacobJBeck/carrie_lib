// Copyright 2016 Carrie Rebhuhn
#include "MultiagentNE.h"
#include <stdio.h>
#include <vector>

using std::vector;

MultiagentNE::MultiagentNE(size_t num_agents, size_t num_inputs,
    size_t num_hidden, size_t num_outputs) {
    std::printf("Creating a MultiagentNE object.\n");
    for (size_t i = 0; i < num_agents; i++) {
        agents.push_back(new NeuroEvo(num_agents, num_inputs, num_hidden,
            num_outputs));
    }
}

MultiagentNE::MultiagentNE(IDomainStateful* domain):
    MultiagentNE(domain->get_n_agents(), 20, domain->get_nn_inputs(),
            domain->get_nn_outputs()) {}

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
