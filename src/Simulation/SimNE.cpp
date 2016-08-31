// Copyright 2016 Carrie Rebhuhn
#include "SimNE.h"
#include <vector>

using std::vector;

SimNE::SimNE(IDomainStateful* domain, MultiagentNE* MAS) :
    ISimulator(domain, MAS), MAS(MAS)
{}

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

void SimNE::runExperimentDifferenceReplay() {
    for (int ep = 0; ep < n_epochs; ep++) {
        time_t epoch_start = time(NULL);
        this->epoch_difference_replay(ep);
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

void SimNE::runExperimentDifference() {
    for (int ep = 0; ep < n_epochs; ep++) {
        time_t epoch_start = time(NULL);
        this->epoch_difference(ep);
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

void SimNE::run_simulation(bool log, matrix3d& recorded, int suppressed) {
    //! Turns recording on if recorded actions passed in empty
    bool recording_on = recorded.empty();

    while (domain->step()) {
        matrix2d A;
        if (recording_on) {
            A = this->get_actions();
            recorded.push_back(A);
        } else {
            A = recorded[domain->get_step()];
        }

        if (suppressed >= 0) {
            A[suppressed] = matrix1d(A[suppressed].size(), 1000);
        }
        domain->simulateStep(A);
        if (log)
            domain->logStep();
    }
}

void SimNE::run_simulation(bool log, int suppressed_agent) {
    while (domain->step()) {
        matrix2d A = this->get_actions();

        if (suppressed_agent >= 0) {
            A[suppressed_agent] = matrix1d(A[suppressed_agent].size(), 1000);
        }
        domain->simulateStep(A);

        if (log)
            domain->logStep();
    }
}

void SimNE::epoch_difference(int ep) {
    printf("Epoch %i", ep);
    SimNE::accounting accounts = SimNE::accounting();
    MAS->generate_new_members();
    do {
        run_simulation(false);

        double G = domain->getPerformance()[0];
        matrix1d D(MAS->agents.size(), 0.0);

        accounts.update(domain->getRewards(), domain->getPerformance());

        domain->reset();
        for (size_t i = 0; i < MAS->agents.size(); i++) {
            //! Suppresses agent i
            run_simulation(false, static_cast<int>(i));
            double Gc = domain->getPerformance()[0];

            D[i] = G - Gc;
            printf("D_%i=%f,", static_cast<int>(i), D[i]);
            domain->reset();
        }
        MAS->update_policy_values(D);
    } while (MAS->set_next_pop_members());
    MAS->select_survivors();

    reward_log.push_back(accounts.best_run);
    metric_log.push_back(accounts.best_run_performance);
}


void SimNE::epoch_difference_replay(int ep) {
    printf("Epoch %i", ep);
    SimNE::accounting accounts = SimNE::accounting();

    MAS->generate_new_members();
    do {
        matrix3d recorded_actions;
        printf("Wait for it...");
        run_simulation(false, recorded_actions);
        printf("waited");

        double G = domain->getPerformance()[0];
        matrix1d D(MAS->agents.size(), 0.0);

        accounts.update(matrix1d(MAS->agents.size(), G),
            matrix1d(MAS->agents.size(), G));

        domain->reset();
        for (size_t i = 0; i < MAS->agents.size(); i++) {
            //! Suppresses agent i during the simulation
            //! Recorded actions played instead of neural network decisions.
            run_simulation(false, recorded_actions, static_cast<int>(i));
            double Gc = domain->getPerformance()[0];

            D[i] = G - Gc;
            printf("D_%i=%f,", static_cast<int>(i), D[i]);
            domain->reset();
        }
        MAS->update_policy_values(D);
    } while (MAS->set_next_pop_members());
    MAS->select_survivors();

    reward_log.push_back(accounts.best_run);
    metric_log.push_back(accounts.best_run_performance);
}

void SimNE::epoch(int ep) {
    bool log = (ep == 0 || ep == n_epochs - 1) ? true : false;

    MAS->generate_new_members();
    SimNE::accounting accounts = SimNE::accounting();

    do {
        // Gets the g
        run_simulation(log);
        matrix1d R = domain->getRewards();
        matrix1d perf = domain->getPerformance();

        accounts.update(R, perf);

        domain->reset();
        MAS->update_policy_values(R);
    } while (MAS->set_next_pop_members());
    MAS->select_survivors();


    reward_log.push_back(accounts.best_run);
    metric_log.push_back(accounts.best_run_performance);
    if (ep == 0)
        domain->exportStepsOfTeam(accounts.best_perf_idx, "untrained");
    if (ep == n_epochs - 1)
        domain->exportStepsOfTeam(accounts.best_perf_idx, "trained");
}

vector<State> SimNE::get_actions() {
    vector<State> S = domain->get_states();
    return MAS->get_actions(S);
}
