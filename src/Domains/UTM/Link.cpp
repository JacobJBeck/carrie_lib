// Copyright 2016 Carrie Rebhuhn
#include "Link.h"

#include <algorithm>
#include <functional>
#include <list>
#include <vector>
#include "STL/include/easystl.h"

using std::vector;
using std::list;
using std::greater;

Link::Link(size_t id, size_t source, size_t target, size_t time,
    vector<size_t> capacity, size_t cardinal_dir, size_t num_types) :
    k_id_(id), k_source_(source), k_target_(target), time_(time),
    k_cardinal_dir_(cardinal_dir), k_capacity_(capacity),
    k_num_types_(num_types), traffic_(size_t(num_types), list<UAV*>())
{}

bool Link::atCapacity(size_t uav_type) {
    return numOverCapacity(uav_type) >= 0;
}

int Link::numOverCapacity(size_t type_id) {
    return static_cast<int>(traffic_[type_id].size() - k_capacity_[type_id]);
}

matrix1d Link::predictedTraversalTime() {
    // Get predicted wait time for each type of UAV
    matrix1d predicted(traffic_.size(), 0.0);
    for (size_t i = 0; i < traffic_.size(); i++) {
        // Collect wait times on all UAVs ON the link
        matrix1d waits;
        for (UAV* u : traffic_[i]) {
            waits.push_back(u->getWait());
        }

        // Sort by wait (descending)
        sort(waits.begin(), waits.end(), greater<double>());

        size_t n_ok = k_capacity_[i] - 1;  // UAVs you don't have to wait for
        size_t n_wait = waits.size() - n_ok;  // UAVs before you in line
        for (int i = 0; i < (n_wait - n_ok); i++) {
            waits.pop_back();
        }

        //if (waits.size() > n_ok)
        //    waits.resize(n_wait);

        // Store predicted link time.
        double w = easymath::sum(waits);
        predicted[i] = time_ + w;
        if (w < 0) {
            printf("bad");
        }
    }
    return predicted;
}

void Link::moveFrom(UAV* u, Link* l) {
    // Add to other list (u is temporarily duplicated)
    add(u);

    // Remove from previous node (l)
    try {
        l->remove(u);
    }
    catch (int e) {
        printf("Exception %i occurred. Pausing then exiting.", e);
        system("pause");
        exit(e);
    }

    // Replan
    u->planAbstractPath();
}

void Link::add(UAV* u) {
    //printf("UAV %i added to link %i.\n", u->getId(), k_id_);
   // system("pause");
    if (time_ < 0) {
        printf("bad");
    }
    u->setWait(time_);
    traffic_.at(STATIC_TYPE).push_back(u);
    //u->setCurLinkId(k_id_);
    //u->setCurSectorId(k_source_);
}


void Link::remove(UAV* u) {
    //printf("UAV %i removed from link %i.\n", u->getId(), k_id_);
//    system("pause");
    try {
        easystl::remove_element(&traffic_[STATIC_TYPE], u);
    }
    catch (int e) {
        throw e;
    }
}

void Link::reset() {
    traffic_ = vector < list<UAV*> >
        (size_t(k_num_types_), list<UAV*>());
}

LinkAgentManager::LinkAgentManager(size_t num_edges, size_t num_types,
    vector<Link*> links, size_t num_state_elements) :
    k_num_edges_(num_edges), k_num_types_(num_types),
    IAgentManager(links.size(), num_state_elements), links_(links)
{
    for (size_t i = 0; i < links.size(); i++) {
        k_link_ids_.insert(std::make_pair(std::make_pair(links[i]->k_source_, links[i]->k_target_), i));
    }
}

matrix2d LinkAgentManager::actionsToWeights(matrix2d agent_actions) {
    matrix2d weights = easymath::zeros(k_num_types_, k_num_edges_);

    for (size_t i = 0; i < k_num_edges_; i++) {
        matrix1d predicted = links_.at(i)->predictedTraversalTime();
        for (size_t t = 0; t < k_num_types_; t++) {
            weights[t][i] = predicted[t] + agent_actions[i][t] * k_alpha_;
            // weights[t][i] = agent_actions[i][t]*1000.0;
        }
    }
    return weights;
}

void LinkAgentManager::addDelay(UAV* u) {
    metrics_.at(getNthLink(u,0)).local_[STATIC_TYPE]++;
}

void LinkAgentManager::addDownstreamDelayCounterfactual(UAV* u) {
    
    // remove the effects of the UAV for the counterfactual..
    // calculate the G that means that the UAV's impact is removed...
    /*
    if (k_square_reward_mode_) {
        // Non-functional, todo
        printf("SQUARED TODO");
        exit(1);
    } else {
        for (size_t i = 0; i < metrics_.size(); i++) {
            if (!u->linkTouched(i)) {
                metrics_[i].g_minus_downstream_[STATIC_TYPE]++;
            } else {
                continue;
            }
        }
    }*/
}

void LinkAgentManager::detectConflicts() {
    for (size_t i = 0; i < links_.size(); i++) {
        for (size_t j = 0; j < links_[i]->traffic_.size(); j++) {
            int over_capacity = links_[i]->numOverCapacity(j);
            if (over_capacity <= 0)
                continue;  // no congestion
            else if (k_square_reward_mode_)
                metrics_[i].local_[j] += over_capacity*over_capacity;
            else
                metrics_[i].local_[j] += over_capacity;
        }
    }
}
