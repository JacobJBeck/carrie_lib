// Copyright 2016 Carrie Rebhuhn
#include "Link.h"

#include <algorithm>
#include <functional>
#include <list>
#include <vector>
#include "STL/easystl.h"

Link::Link(size_t ID, int source_set, int target_set,
    int time, std::vector<size_t> capacity, int cardinal_dir, size_t n_types_set) :
    ID(ID),
    source(source_set),
    target(target_set),
    time(time),
    cardinal_dir(cardinal_dir),
    capacity(capacity),
    n_types(n_types_set),
    traffic(size_t(n_types_set), std::list<UAV*>())
{}

bool Link::at_capacity(size_t UAV_type) {
    return number_over_capacity(UAV_type) >= 0;
}

int Link::number_over_capacity(size_t type_ID) {
    return static_cast<int>(traffic[type_ID].size() - capacity[type_ID]);
}

matrix1d Link::predicted_traversal_time() {
    // Get predicted wait time for each type of UAV
    matrix1d predicted(traffic.size(),0.0);
    for (size_t i = 0; i < traffic.size(); i++) {
        // Collect wait times on all UAVs ON the link
        matrix1d waits;
        for (UAV* u : traffic[i]) {
            waits.push_back(u->get_wait());
        }

        // Sort by wait (descending)
        std::sort(waits.begin(), waits.end(), std::greater<double>());

        size_t n_ok = capacity[i] - 1;  // UAVs you don't have to wait for
        size_t n_wait = waits.size() - n_ok;  // UAVs before you in line
        if (waits.size() > n_ok)
            waits.resize(n_wait);

        // Store predicted link time.
        double w = easymath::sum(waits);
        predicted[i] = time + w;
        if (w < 0) {
            printf("bad");
        }
    }
    return predicted;
}

void Link::move_from(UAV* u, Link* l) {
    // Add to other list (u is temporarily duplicated)
    add(u);

    // Remove from previous node (l)
    l->remove(u);

    // Replan
    u->planAbstractPath();
}

void Link::add(UAV* u){
    if (time < 0) {
        printf("bad");
    }
    u->set_wait(time);
    traffic.at(u->get_type()).push_back(u);
    u->set_cur_link_ID(ID);
    u->set_cur_sector_ID(source);
}


void Link::remove(UAV* u) {
    easystl::remove_element(&traffic[u->get_type()], u);
}

void Link::reset() {
    traffic = std::vector < std::list<UAV*> >
        (size_t(n_types), std::list<UAV*>());
}

LinkAgentManager::LinkAgentManager(int n_edges, int n_types,
    std::vector<Link*> links, size_t n_state_elements_set) :
    n_edges(n_edges), n_types(n_types), IAgentManager(links.size(),n_state_elements_set), links(links)
{};

matrix2d LinkAgentManager::actions2weights(matrix2d agent_actions) {
    matrix2d weights = easymath::zeros(n_types, n_edges);
    
    for (int i = 0; i < n_edges; i++) {
        matrix1d predicted = links.at(i)->predicted_traversal_time();
        for (int t = 0; t < n_types; t++) {
            weights[t][i] = predicted[t] + agent_actions[i][t] * alpha;
            // weights[t][i] = agent_actions[i][t]*1000.0;
        }
    }
    return weights;
}

void LinkAgentManager::add_delay(UAV* u) {
    metrics.at(u->get_cur_link()).local[u->get_type()]++;
}

void LinkAgentManager::add_downstream_delay_counterfactual(UAV* u) {
    // remove the effects of the UAV for the counterfactual..
    // calculate the G that means that the UAV's impact is removed...

    if (square_reward) {
        // Non-functional, todo
        printf("SQUARED TODO");
        exit(1);
    } else {
        for (size_t i = 0; i < metrics.size(); i++) {
            if (!u->link_touched(i)) {
                metrics[i].G_minus_downstream[u->get_type()]++;
            } else {
                continue;
            }
        }
    }
}

void LinkAgentManager::detect_conflicts() {
    for (size_t i = 0; i < links.size(); i++) {
        for (size_t j = 0; j < links[i]->traffic.size(); j++) {
            int over_capacity = links[i]->number_over_capacity(j);
            if (over_capacity <= 0)
                continue;  // no congestion
            else if (square_reward)
                metrics[i].local[j] += over_capacity*over_capacity;
            else
                metrics[i].local[j] += over_capacity;
        }
    }
}
