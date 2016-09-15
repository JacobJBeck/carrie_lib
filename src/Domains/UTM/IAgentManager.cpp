// Copyright 2016 Carrie Rebhuhn
#include "IAgentManager.h"
#include <string>
#include <vector>
#include "UTMModesAndFiles.h"

using std::string;
using std::vector;
using std::runtime_error;
using easymath::sum;
using easymath::zeros;
using easymath::square;
using easymath::operator+;
using easymath::operator/;
using easymath::operator-;

IAgentManager::IAgentManager(size_t num_agents, size_t num_state_elements):
    k_num_state_elements_(num_state_elements)
{
    YAML::Node config = YAML::LoadFile("config.yaml");
    k_square_reward_mode_ = config["modes"]["square"].as<bool>();
    k_alpha_ = config["constants"]["alpha"].as<double>();
    std::string rwd = config["modes"]["reward"].as<std::string>();
    size_t num_types = UTMModes::getNumTypes(config);
    metrics_ = vector<Reward_Metrics>(num_agents, Reward_Metrics(num_types));

    try {
        if (rwd == "difference_avg") {
            counterfactual = &IAgentManager::GcAverage;
        } else if (rwd == "difference_downstream") {
            counterfactual = &IAgentManager::GcDownstream;
        } else if (rwd == "difference_realloc") {
            counterfactual = &IAgentManager::GcRealloc;
        } else if (rwd == "difference_touched") {
            counterfactual = &IAgentManager::GcTouched;
        } else if (rwd == "global") {
            counterfactual = &IAgentManager::Gc0;
        } else {
            throw runtime_error("Bad reward mode.");
        }
    }
    catch (runtime_error) {
        printf("Bad reward mode!");
        exit(1);
    }
}

matrix1d IAgentManager::global() {
    double sum = 0.0;
    for (Reward_Metrics r : metrics_) {
        sum += easymath::sum(r.local_);
    }
    return matrix1d(metrics_.size(), -sum);
}

matrix1d IAgentManager::GcAverage() {
    matrix1d G_c = zeros(metrics_.size());
    for (size_t i = 0; i < metrics_.size(); i++)
        G_c[i] = -sum(metrics_[i].g_avg_);

    return G_c;
}

void IAgentManager::addAverageCounterfactual() {
    // This actually is a local reward
    /*size_t n_types = metrics[0].local.size();
    size_t n_agents = metrics.size();

    for (size_t i = 0; i < metrics.size(); i++) {
        matrix1d m = zeros(n_types);
        for (size_t j = 0; j < metrics.size(); j++) {
            if (i != j)
                m = m + metrics[i].local;
            else
                m = m + (metrics[i].local / (*steps));
        }
        metrics[i].G_avg = m;
    }*/
}

matrix1d IAgentManager::GcDownstream() {
    matrix1d G_c = zeros(metrics_.size());
    for (size_t i = 0; i < metrics_.size(); i++) {
        G_c[i] = -sum(metrics_[i].g_minus_downstream_);
    }
    return G_c;
}

matrix1d IAgentManager::GcRealloc() {
    matrix1d G_c = zeros(metrics_.size());
    for (size_t i = 0; i < metrics_.size(); i++) {
        G_c[i] = -sum(metrics_[i].g_random_realloc_);
    }
    return G_c;
}

matrix1d IAgentManager::GcTouched() {
    matrix1d G_c = zeros(metrics_.size());
    for (size_t i = 0; i < metrics_.size(); i++) {
        G_c[i] = -sum(metrics_[i].g_touched_);
    }
    return G_c;
}

matrix1d IAgentManager::Gc0() {
    return zeros(metrics_.size());
}

matrix1d IAgentManager::performance() {
    matrix1d G = global();
    if (k_square_reward_mode_)
        square(&G);
    return G;
}

matrix1d IAgentManager::reward() {
    matrix1d Gc = (this->*counterfactual)();
    matrix1d G = global();

    if (k_square_reward_mode_) {
        square(&Gc);
        square(&G);
    }
    return G - Gc;
}

void IAgentManager::logAgentActions(matrix2d agent_step_actions) {
    agent_actions_.push_back(agent_step_actions);
}

bool IAgentManager::lastActionDifferent() {
    if (agent_actions_.size() > 1) {
        matrix2d last_action = agent_actions_.back();
        matrix2d cur_action = agent_actions_[agent_actions_.size() - 2];

        return last_action != cur_action;
    }
    return true;
}

void IAgentManager::exportAgentActions(int fileID) {
    string actionfile = "actions-" + std::to_string(fileID) + ".csv";
    string statefile = "states-" + std::to_string(fileID) + ".csv";
    FileOut::print_vector(agent_actions_, actionfile);
    FileOut::print_vector(agent_states_, statefile);
}

void IAgentManager::reset() {
    agent_actions_.clear();
    agent_states_.clear();
    size_t n_agents = metrics_.size();
    size_t n_types = metrics_[0].local_.size();
    metrics_ = std::vector<Reward_Metrics>(n_agents, Reward_Metrics(n_types));
}
