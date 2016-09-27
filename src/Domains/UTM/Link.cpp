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
    size_t capacity, size_t cardinal_dir) :
    k_id_(id), k_source_(source), k_target_(target), time_(time),
    k_cardinal_dir_(cardinal_dir), k_capacity_(capacity),
    traffic_(list<UAV*>())
{}

bool Link::atCapacity() {
    return numOverCapacity() >= 0;
}

int Link::numOverCapacity() {
    return static_cast<int>(traffic_.size() - k_capacity_);
}

double Link::predictedTraversalTime() {
    // Get predicted wait time for each type of UAV
    double predicted = 0;
    
    // Collect wait times on all UAVs ON the link
    matrix1d waits;
    for (UAV* u : traffic_) {
        waits.push_back(u->getWait());
    }

    // Sort by wait (descending)
    sort(waits.begin(), waits.end(), greater<double>());

    size_t n_ok = k_capacity_ - 1;  // UAVs you don't have to wait for
    size_t n_wait = waits.size() - n_ok;  // UAVs before you in line
    for (int i = 0; i < (n_wait - n_ok); i++) {
        waits.pop_back();
    }

    // Store predicted link time.
    double w = easymath::sum(waits);
    predicted = time_ + w;
    if (w < 0) {
        printf("bad");
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
    traffic_.push_back(u);
}


void Link::remove(UAV* u) {
    try {
        easystl::remove_element(&traffic_, u);
    }
    catch (int e) {
        throw e;
    }
}

void Link::reset() {
    traffic_ = list<UAV*>();
}

LinkAgentManager::LinkAgentManager(size_t num_edges,
    vector<Link*> links, size_t num_state_elements) :
    k_num_edges_(num_edges),
    IAgentManager(links.size(), num_state_elements), links_(links)
{
    for (size_t i = 0; i < links.size(); i++) {
        k_link_ids_.insert(std::make_pair(std::make_pair(links[i]->k_source_, links[i]->k_target_), i));
    }
}

matrix1d LinkAgentManager::actionsToWeights(matrix2d agent_actions) {
    matrix1d weights = easymath::zeros(k_num_edges_);

    for (size_t i = 0; i < k_num_edges_; i++) {
        double predicted = links_.at(i)->predictedTraversalTime();
        weights[i] = predicted + agent_actions[i][0] * k_alpha_; // only one action
    }
    return weights;
}

void LinkAgentManager::addDelay(UAV* u) {
    metrics_.at(getNthLink(u,0)).local_++;
}

void LinkAgentManager::addDownstreamDelayCounterfactual(UAV* u) {
}

void LinkAgentManager::detectConflicts() {
    for (size_t i = 0; i < links_.size(); i++) {
        int over_capacity = links_[i]->numOverCapacity();
        if (over_capacity <= 0)
            continue;  // no congestion
        else if (k_square_reward_mode_)
            metrics_[i].local_ += over_capacity*over_capacity;
        else
            metrics_[i].local_ += over_capacity;

    }
}
