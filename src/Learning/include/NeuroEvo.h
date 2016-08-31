// Copyright 2016 Carrie Rebhuhn
#ifndef SRC_LEARNING_NEUROEVO_H_
#define SRC_LEARNING_NEUROEVO_H_

#include <set>
#include <utility>
#include <algorithm>
#include <list>
#include <string>
#include <vector>

#include "NeuralNet.h"
#include "Evolution.h"
#include "FileIO/include/FileIn.h"
#include "FileIO/include/FileOut.h"

class NeuroEvoParameters {
 public:
    NeuroEvoParameters(size_t inputSet, size_t outputSet);
    static const int nHidden = 50;
    static const int popSize = 10;  // surviving population size

    size_t nInput, nOutput;
    double epsilon;  // for epsilon-greedy selection: currently unused
};


class NeuroEvo : public Evolution<NeuralNet> {
 public:
     //! Life cycle
    NeuroEvo() {}
    explicit NeuroEvo(NeuroEvoParameters* neuroEvoParamsSet);
    ~NeuroEvo(void) { deletePopulation(); }
    void deep_copy(const NeuroEvo &NE);
    void deletePopulation();

    //! Class variables
    NeuroEvoParameters* params;
    std::list<NeuralNet*> population;
    std::list<NeuralNet*>::iterator pop_member_active;

    //! Mutators
    void generate_new_members();
    bool select_new_member();
    void select_survivors();
    void update_policy_values(double R);
    void load(std::string filein);

    //! Accessors
    double getBestMemberVal();
    static bool NNCompare(const NeuralNet *x, const NeuralNet *y) {
        return (x->get_evaluation() > y->get_evaluation());
    }
    Action get_action(State state);
    Action get_action(std::vector<State> state);
    void save(std::string fileout);
};
#endif  // SRC_LEARNING_NEUROEVO_H_
