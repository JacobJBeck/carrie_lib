// Copyright 2016 Carrie Rebhuhn
#ifndef SRC_DOMAINS_ROVER_ROVERDOMAIN_H_
#define SRC_DOMAINS_ROVER_ROVERDOMAIN_H_
#endif  // SRC_DOMAINS_ROVER_ROVERDOMAIN_H_

#pragma once
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <string>
#include "Domains/IDomainStateful.h"
#include "Learning\include\NeuralNet.h"

#define MAXPOI 100.0
#define XDIM 100.0
#define YDIM 100.0
#define NPOIS 10
#define NROVS 10
#define RAND static_cast<double>(rand())/static_cast<double>(RAND_MAX)

typedef size_t uint;

template <class L, class R>
double ifLeftAddRight(L l, R r) {
    double sum = 0.0;
    auto lb = l.begin();
    auto le = l.end();
    auto rb = r.begin();
    while (lb != le) {
        if (*lb) sum += *rb;
        lb++;
        rb++;
    }
    return sum;
}

void append(matrix1d &a, const matrix1d &b);

//! Class for xy locations
class Loc {
 public:
    Loc(double x, double y) : x_(x), y_(y) {}
    double x_, y_;
    int relQuadrant(Loc* l);
    double norm(Loc *l);
    void operator+=(std::vector<double> v);
};

class POI {
 public:
    POI() : value_(RAND*MAXPOI), l_(new Loc(RAND*XDIM, RAND*YDIM)) {}
    ~POI() { delete l_; }
    double value_;
    Loc* l_;
};

typedef std::vector<Loc*> Locs;
typedef std::vector<double*> Vals;

class Sensor {
 public:
    Sensor(Loc* loc, int q, double dmin = 1.0) : dmin_(dmin),
        dmin_sq_(dmin*dmin), l_(loc), quadrant_(q) {}
    int quadrant_;

    Locs L_;  // all sensed locations
    Loc* l_;   // my location

    template <class T>
    static Locs getLocs(const std::vector<T> &obj) {
        Locs L;
        for (T o : obj)
            L.push_back(o->l_);
        return L;
    }

    void initialize(Locs L) { L_ = L; }
    double denom(const std::vector<bool> &in_quadrant);
    virtual double numer(const std::vector<bool> &in_quadrant) = 0;
    double sense();

    double dmin_, dmin_sq_;
    // Squared euclidean norm, bounded by a minimum observation value
    double delta(Loc* l) { return std::max(l_->norm(l), dmin_sq_); }
    std::vector<bool> inQuadrant(Locs L);
    bool inQuadrant(Loc* l) { return  l_->relQuadrant(l) == quadrant_; }
};

class POISensor : public Sensor {
 public:
    POISensor(Loc* l, int q) : Sensor(l, q) {}
    std::vector<double> value_;
    double numer(const std::vector<bool> &in_quadrant) {
        return ifLeftAddRight(in_quadrant, value_);
    }
};

class RoverSensor : public Sensor {
 public:
    RoverSensor(Loc* l, int q) : Sensor(l, q) {}
    double numer(const std::vector<bool> &in_quadrant) {
        return static_cast<double>(std::count(in_quadrant.begin(),
            in_quadrant.end(), true));
    }
};

class Rover {
 public:
    Rover();
    ~Rover() { delete l_; }
    std::vector<POISensor> psensor;
    std::vector<RoverSensor> rsensor;
    Loc *l_;
};

/*class IReward{
 public:
    IReward(IDomainStateful* domain): domain_(domain) {
        k_num_agents_ = domain->getNumAgents();
    }
    size_t k_num_agents_;
    IDomainStateful* domain_; 

    virtual void getRewardName() = 0;
    virtual void getRewardStateInfo() = 0;
    virtual matrix1d operator()() = 0;
};

class RoverGlobalReward : public IReward {
    RoverGlobalReward(IDomainStateful* domain) : IReward(domain),
        global_t_(matrix1d(domain->getNumSteps(), 0.0)) {}
    matrix1d global_t_; // global reward at each timestep
    
    void getRewardStateInfo() {
        // get all the information from the sensors+everything used to calculate stuff
    }
};*/

class RoverDomain : public IDomainStateful {
 public:
    std::string reward_mode_;
    std::vector<Loc> poi_start_locs_;
    std::vector<Loc> rover_start_locs_;
    std::vector<POI*> pois;
    std::vector<Rover*> rovers;
    RoverDomain();
    ~RoverDomain();
    matrix2d getStates();
    matrix1d getRewards() {
        if (reward_mode_ == "global")
            return matrix1d(k_num_agents_, G());
        else if (reward_mode_ == "difference")
            return D_gl();
        else {
            std::cout << "No reward mode set.";
            exit(1);
            return matrix1d();
        }
    }

    matrix1d getPerformance() { return matrix1d(NROVS, g_cumulative_); }
    void logStep() {}

    void initialize();
    double mindist(POI *p);

    // Global reward at a particular timestep
    double Gt();
    double G() {
        return g_cumulative_;
    }
    
    // todo: populate state_history
    matrix2d state_history_; // agent, {state, timestep} (appended each step)
    matrix2d action_history_;
    std::vector<NeuralNet*> g_;
    matrix1d G_local() {
        matrix1d g(k_num_agents_, 0.0);
        for (size_t i = 0; i < k_num_agents_; i++) {
            matrix1d s = state_history_[i];
            s.resize(s.size() + action_history_.size(), 0.0); // zero action counterfactual
            g[i] = (*g_[i])(s)[0]; // first index of output (output should be 1 element)
        }
        return g;
    }
    matrix1d D_gl() {
        return matrix1d(k_num_agents_, G()) - G_local();
    }

    matrix2d getStateActionHistory() {
        matrix2d SA(k_num_agents_);
        for (size_t i = 0; i < k_num_agents_; i++) {
            SA[i] = state_history_[i];
            append(SA[i], action_history_[i]);
        }
        return SA;
    }

    matrix1d G_log;
    matrix3d state_action_log_;

    void resetSAG() {
        state_history_ = matrix2d(k_num_agents_);
        action_history_ = matrix2d(k_num_agents_); // append each time an action taken
    }


    void simulateStep(matrix2d A);
    std::string createExperimentDirectory(std::string s) { return ""; }
    double g_cumulative_;
    void reset();
};

