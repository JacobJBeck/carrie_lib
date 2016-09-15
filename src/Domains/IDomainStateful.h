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
    explicit IDomainStateful(std::string config_file);
    virtual ~IDomainStateful(void);

    // Returns the state vector for the set of agents, [AGENTID][STATEELEMENT]
    virtual matrix2d getStates() = 0;

    //! [AGENTID][TYPEID][STATEELEMENT]
    virtual matrix3d getTypeStates() = 0;

    //! Returns the reward vector for a set of agents [AGENTID]
    virtual matrix1d getRewards() = 0;

    //! Returns the performance vector for a set of agents
    virtual matrix1d getPerformance() = 0;
    virtual void simulateStep(matrix2d agent_action) = 0;
    virtual void reset() = 0;
    virtual void logStep() = 0;
    virtual void exportStepsOfTeam(int team, std::string suffix) = 0;

    //! Creates a directory for the current domain's parameters
    virtual std::string createExperimentDirectory(std::string config_file) = 0;

    size_t getNumAgents() const { return k_num_agents_; }
    size_t getNumNNInputs() const { return k_num_state_elements_; }
    size_t getNumNNOutputs() const { return k_num_control_elements_; }
    size_t getStep() const { return *cur_step_; }
    bool step() {
        if (*cur_step_ >= k_num_steps_)
            return false;
        (*cur_step_)++;
        return true;
    }

 protected:
    size_t * cur_step_, k_num_state_elements_, k_num_control_elements_,
        k_num_steps_, k_num_types_, k_num_agents_;   // agents determined later!
};

#endif  // SRC_DOMAINS_IDOMAINSTATEFUL_H_
