// Copyright 2016 Carrie Rebhuhn
#include "SimNE.h"

// C
#include <time.h>
// C++
#include <vector>
#include <limits>
#include <iostream>

#include "Multiagent/include/MultiagentNE.h"

using std::vector;

SimNE::SimNE(IDomainStateful* domain, MultiagentNE* MAS) :
    ISimulator<NeuroEvo>(domain, MAS), MAS(MAS)
{
    turnOnEvo();
}

SimNE::~SimNE(void) {}

void SimNE::runExperiment() {
    for (int ep = 0; ep < n_epochs; ep++) {
        time_t epoch_start = time(NULL);
        this->epoch(ep);
        time_t epoch_end = time(NULL);
        time_t epoch_time = epoch_end - epoch_start;
        time_t run_time_left = (time_t(n_epochs - ep))*epoch_time;
        time_t run_end_time = epoch_end + run_time_left;

        char end_clock_time[26];
#ifdef _WIN32
        ctime_s(end_clock_time, sizeof(end_clock_time), &run_end_time);
#endif

        printf("Epoch %i took %i seconds.\n", ep, static_cast<int>(epoch_time));
        std::cout << "Estimated run end time: " << end_clock_time << std::endl;
    }
}

void SimNE::runSimulation(bool log) {
    do{
        matrix2d A = this->getActions();
        domain->simulateStep(A);

        if (log)
            domain->logStep();
    } while (domain->step());
}

void SimNE::epoch(int ep) {
    bool log = (ep == 0 || ep == n_epochs - 1) ? true : false;

    if (!no_update_) {
        MAS->generate_new_members();
    }
    SimNE::accounting accounts = SimNE::accounting();

    do {
        // Gets the g
        runSimulation(log);
        matrix1d R = domain->getRewards();
        matrix1d perf = domain->getPerformance();

        accounts.update(R, perf);

        domain->reset();
        MAS->update_policy_values(R);
    } while (MAS->set_next_pop_members());
    if (!no_update_) {
        MAS->select_survivors();
    }


    reward_log.push_back(accounts.best_run);
    metric_log.push_back(accounts.best_run_performance);
}

vector<State> SimNE::getActions() {
    vector<State> S = domain->getStates();
    return MAS->getActions(S);
}

SimNE::accounting::accounting() {
    best_run = -std::numeric_limits<double>::max();
    best_run_performance = -std::numeric_limits<double>::max();
    n = 0;
    best_perf_idx = 0;
}

void SimNE::accounting::update(const vector<double> &R,
    const vector<double> &perf) {
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
