// Copyright 2016 Carrie Rebhuhn
#ifndef DOMAINS_IDOMAINSTATEFUL_H_
#define DOMAINS_IDOMAINSTATEFUL_H_

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
    virtual matrix2d get_states() = 0;

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

    size_t get_n_agents() const { return n_agents; }
    size_t get_nn_inputs() const { return n_state_elements; }
    size_t get_nn_outputs() const { return n_control_elements; }
    size_t get_step() const { return *cur_step; }
    bool step() {
        if (*cur_step >= n_steps)
            return false;
        (*cur_step) ++;
        return true;
    }

 protected:
    size_t * cur_step;
    size_t n_state_elements;
    size_t n_control_elements;
    size_t n_steps;
    size_t n_types;
    size_t n_agents;   // agents determined later!

};

#endif  // DOMAINS_IDOMAINSTATEFUL_H_
