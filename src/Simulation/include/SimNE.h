// Copyright 2016 Carrie Rebhuhn
#ifndef SIMULATION_SIMNE_H_
#define SIMULATION_SIMNE_H_

// C
#include <time.h>

// C++
#include <sstream>
#include <limits>

// Libraries
#include "ISimulator.h"
#include "Multiagent/include/MultiagentNE.h"
#include "Math/include/easymath.h"

class SimNE : public ISimulator {
 public:
    SimNE(IDomainStateful* domain, MultiagentNE* MAS);
    MultiagentNE* MAS;
    virtual ~SimNE(void);

    static const int n_epochs = 100;
    static const int n_trials = 1;
    size_t* step;  // step counter for running the simulation

    virtual void runExperiment();
    virtual void epoch(int ep);

    void epochDifference(int ep);
    void epochDifferenceReplay(int ep);
    void runSimulation(bool log, int suppressed_agent = -1);
    void runSimulation(bool log, matrix3d& actions_recorded, int suppressed_agent = -1);

    //! Gets actions based on current state: OVERLOAD FOR TYPES
    virtual std::vector<Action> getActions();
    void runExperimentDifference();
    void runExperimentDifferenceReplay();
    struct accounting {
        accounting() {
            best_run = -std::numeric_limits<double>::max();
            best_run_performance = -std::numeric_limits<double>::max();
            n = 0;
            best_perf_idx = 0;
        }

        void update(const matrix1d &R, const matrix1d &perf) {
            double avg_G = easymath::mean(R);
            double avg_perf = easymath::mean(perf);

            if (avg_G > best_run) {
                best_run = avg_G;
            }
            if (avg_perf > best_run_performance) {
                best_run_performance = avg_perf;
                best_perf_idx = n;
            }

            printf("NN#%i, %f, %f, %f\n", n, best_run_performance,
                best_run, avg_perf);
        }
        int n, best_perf_idx;
        double best_run, best_run_performance;
    };
};
#endif  // SIMULATION_SIMNE_H_
