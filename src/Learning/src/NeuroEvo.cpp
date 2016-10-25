//! Copyright 2016 Carrie Rebhuhn
#include "NeuroEvo.h"
#include <algorithm>
#include <list>
#include <string>
#include <vector>

using std::list;
using std::vector;

matrix1d operator*(const double A, matrix1d B) {
    for (auto &b : B) {
        b *= A;
    }
    return B;
}

matrix2d operator*(const double A, matrix2d B) {
        for (auto &b : B) {
            b = A*b;
        }
        return B;
    }

NeuroEvo::NeuroEvo(size_t num_input, size_t num_hidden, size_t num_output,
    size_t population_size) : k_population_size_(population_size) {
    for (size_t i = 0; i <population_size; i++) {
        NeuralNet* nn = new NeuralNet(num_input, num_hidden, num_output);
        population_.push_back(nn);
    }
    pop_member_active_ = population_.begin();
}

void NeuroEvo::updatePolicyValues(double R) {
    // Add together xi values, for averaging
    double xi = 0.1;  // "learning rate" for NE
    double V = (*pop_member_active_)->getEvaluation();
    V = xi*(R - V) + V;
    (*pop_member_active_)->update(V);
}

NeuroEvo::Action NeuroEvo::getAction(NeuroEvo::State state) {
    return (**pop_member_active_)(state);
}

NeuroEvo::Action NeuroEvo::getAction(std::vector<NeuroEvo::State> state) {
    State stateSum(state[0].size(), 0.0);

    // state[type][state_element] -- specifies combination for state
    for (size_t i = 0; i < state.size(); i++) {
        for (size_t j = 0; j < state[i].size(); j++) {
            stateSum[j] += state[i][j];
        }
    }

    return getAction(stateSum);
}

void NeuroEvo::deletePopulation() {
    while (!population_.empty()) {
        delete population_.back();
        population_.pop_back();
    }
}


bool NeuroEvo::selectNewMember() {
    ++pop_member_active_;
    if (pop_member_active_ == population_.end()) {
        pop_member_active_ = population_.begin();
        return false;
    } else {
        return true;
    }
}

void NeuroEvo::generateNewMembers() {
    // Mutate existing members to generate more
    list<NeuralNet*>::iterator popMember = population_.begin();
    for (size_t i = 0; i < k_population_size_; i++) {  // add new members
        // commented out so that you take parent's evaluation
        // (*popMember)->evaluation = 0.0;
        // dereference pointer AND iterator
        NeuralNet* m = new NeuralNet(**popMember);
        m->mutate();
        population_.push_back(m);
        ++popMember;
    }
}

double NeuroEvo::getBestMemberVal() {
    // Find the HIGHEST FITNESS value of any neural network
    double highest = population_.front()->getEvaluation();
    for (NeuralNet* p : population_) {
        if (highest < p->getEvaluation()) highest = p->getEvaluation();
    }
    return highest;
}

void random_shuffle(list<NeuralNet*> *L) {
    vector<NeuralNet*> tmp(L->begin(), L->end());
    random_shuffle(tmp.begin(), tmp.end());
    copy(tmp.begin(), tmp.end(), L->begin());
}

void NeuroEvo::selectSurvivors() {
    // Select neural networks with the HIGHEST FITNESS
    population_.sort(NNCompare);  // Sort by the highest fitness
    size_t nExtraNN = population_.size() - k_population_size_;
    for (size_t i = 0; i < nExtraNN; i++) {  // Remove the extra
        delete population_.back();
        population_.pop_back();
    }
    random_shuffle(&population_);

    pop_member_active_ = population_.begin();
}

void NeuroEvo::deepCopy(const NeuroEvo &NE) {
    // Creates new pointer addresses for the neural nets
    k_population_size_ = NE.k_population_size_;

    deletePopulation();
    for (NeuralNet* p : NE.population_) {
        population_.push_back(new NeuralNet(*p));
    }
}


void NeuroEvo::save(std::string fileout) {
    for (NeuralNet* p : population_) {
        p->save(fileout);
    }
}

void NeuroEvo::load(std::string filein) {
    matrix2d netinfo = cio::read2<double>(filein);

    int i = 0;
    for (NeuralNet* p : population_) {
        // assume that population_ already has the correct size
        p->load(netinfo[i], netinfo[i + 1]);
        i += 2;
    }
}
