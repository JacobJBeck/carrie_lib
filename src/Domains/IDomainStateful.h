// Copyright 2016 Carrie Rebhuhn
#ifndef SRC_DOMAINS_IDOMAINSTATEFUL_H_
#define SRC_DOMAINS_IDOMAINSTATEFUL_H_

#include <vector>
#include <string>

typedef std::vector<double> matrix1d;
typedef std::vector<matrix1d> matrix2d;
typedef std::vector<matrix2d> matrix3d;

class IDomainStateful {
 public:
    IDomainStateful();
    IDomainStateful(size_t num_states, size_t num_actions, size_t num_agents, size_t num_steps) :
        cur_step_(new size_t(0)), k_num_actions_(num_actions), k_num_agents_(num_agents), k_num_states_(num_states), k_num_steps_(num_steps) {
        (*cur_step_) = 0;
    }
    IDomainStateful(std::string config_file);

    virtual ~IDomainStateful(void) {
        delete cur_step_;
    };

    // Returns the state vector for the set of agents, [AGENTID][STATEELEMENT]
    virtual matrix2d getStates() = 0;

    //! Returns the reward vector for a set of agents [AGENTID]
    virtual matrix1d getRewards() = 0;

    //! Returns the performance vector for a set of agents
    virtual matrix1d getPerformance() = 0;
    virtual void simulateStep(matrix2d agent_action) = 0;
    virtual void reset() = 0;
    virtual void logStep() = 0;

    //! Creates a directory for the current domain's parameters
    virtual std::string createExperimentDirectory(std::string config_file) = 0;

    size_t getNumSteps() const { return k_num_steps_; }
    size_t getNumAgents() const { return k_num_agents_; }
    size_t getNumNNInputs() const { return k_num_states_; }
    size_t getNumNNOutputs() const { return k_num_actions_; }
    size_t getStep() const { return *cur_step_; }
    bool step() {
        if (*cur_step_ < k_num_steps_-1) {
            (*cur_step_)++;
            return true;
        }
        return false;
    }

 protected:
    size_t * cur_step_, k_num_states_, k_num_actions_,
        k_num_steps_, k_num_agents_;   // agents determined later!
};

#endif  // SRC_DOMAINS_IDOMAINSTATEFUL_H_
