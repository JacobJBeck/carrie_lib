//! Copyright 2016 Carrie Rebhuhn
#include "NeuroEvo.h"
#include <algorithm>
#include <list>
#include <string>
#include <vector>

using std::list;
using std::vector;

void NeuroEvo::update_policy_values(double R) {
    // Add together xi values, for averaging
    double xi = 0.1;  // "learning rate" for NE
    double V = (*pop_member_active)->getEvaluation();
    V = xi*(R - V) + V;
    (*pop_member_active)->update(V);
}

NeuroEvo::Action NeuroEvo::get_action(NeuroEvo::State state) {
    return (**pop_member_active)(state);
}

NeuroEvo::Action NeuroEvo::get_action(std::vector<NeuroEvo::State> state) {
    State stateSum(state[0].size(), 0.0);

    // state[type][state_element] -- specifies combination for state
    for (size_t i = 0; i < state.size(); i++) {
        for (size_t j = 0; j < state[i].size(); j++) {
            stateSum[j] += state[i][j];
        }
    }

    return get_action(stateSum);
}

void NeuroEvo::deletePopulation() {
    while (!population.empty()) {
        delete population.back();
        population.pop_back();
    }
}


bool NeuroEvo::select_new_member() {
    ++pop_member_active;
    if (pop_member_active == population.end()) {
        pop_member_active = population.begin();
        return false;
    } else {
        return true;
    }
}

void NeuroEvo::generate_new_members() {
    // Mutate existing members to generate more
    list<NeuralNet*>::iterator popMember = population.begin();
    for (size_t i = 0; i < k_population_size_; i++) {  // add new members
        // commented out so that you take parent's evaluation
        // (*popMember)->evaluation = 0.0;
        // dereference pointer AND iterator
        NeuralNet* m = new NeuralNet(**popMember);
        m->mutate();
        population.push_back(m);
        ++popMember;
    }
}

double NeuroEvo::getBestMemberVal() {
    // Find the HIGHEST FITNESS value of any neural network
    double highest = population.front()->getEvaluation();
    for (NeuralNet* p : population) {
        if (highest < p->getEvaluation()) highest = p->getEvaluation();
    }
    return highest;
}

void random_shuffle(list<NeuralNet*> *L) {
    vector<NeuralNet*> tmp(L->begin(), L->end());
    random_shuffle(tmp.begin(), tmp.end());
    copy(tmp.begin(), tmp.end(), L->begin());
}

void NeuroEvo::select_survivors() {
    // Select neural networks with the HIGHEST FITNESS
    population.sort(NNCompare);  // Sort by the highest fitness
    size_t nExtraNN = population.size() - k_population_size_;
    for (size_t i = 0; i < nExtraNN; i++) {  // Remove the extra
        delete population.back();
        population.pop_back();
    }
    random_shuffle(&population);

    pop_member_active = population.begin();
}


void NeuroEvo::deep_copy(const NeuroEvo &NE) {
    // Creates new pointer addresses for the neural nets
    k_population_size_ = NE.k_population_size_;

    deletePopulation();
    for (NeuralNet* p : NE.population) {
        population.push_back(new NeuralNet(*p));
    }
}


void NeuroEvo::save(std::string fileout) {
    for (NeuralNet* p : population) {
        p->save(fileout);
    }
}

void NeuroEvo::load(std::string filein) {
    matrix2d netinfo = easyio::read2<double>(filein);

    int i = 0;
    for (NeuralNet* p : population) {
        // assume that population already has the correct size
        p->load(netinfo[i], netinfo[i + 1]);
        i += 2;
    }
}
