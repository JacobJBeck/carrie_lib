// Copyright 2016 Carrie Rebhuhn
#ifndef SRC_SIMULATION_INCLUDE_SIMNE_H_
#define SRC_SIMULATION_INCLUDE_SIMNE_H_

// C++
#include <vector>

// Libraries
#include "ISimulator.h"
#include "Multiagent/include/MultiagentNE.h"

class SimNE : public ISimulator<NeuroEvo> {
 public:
    SimNE(IDomainStateful* domain, MultiagentNE* MAS);
    MultiagentNE* MAS;
    virtual ~SimNE(void);
    bool no_update_;
    void turnOffEvo() { no_update_ = true; }
    void turnOnEvo() { no_update_ = false; }

    static const int n_epochs = 100;
    static const int n_trials = 1;
    size_t* step;  // step counter for running the simulation


    //! Gets actions based on current state: OVERLOAD FOR TYPES
    virtual std::vector<Action> getActions();

    virtual void epoch(int ep);
    void runSimulation(bool log);

    virtual void runExperiment();

    struct accounting {
        accounting();
        void update(const std::vector<double> &R, const std::vector<double> &perf);
        int n, best_perf_idx;
        double best_run, best_run_performance;
    };
};
#endif  // SRC_SIMULATION_INCLUDE_SIMNE_H_
