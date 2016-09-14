// Copyright 2016 Carrie Rebhuhn
#ifndef SRC_LEARNING_INCLUDE_NEUROEVO_H_
#define SRC_LEARNING_INCLUDE_NEUROEVO_H_

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

class NeuroEvo : public Evolution<NeuralNet> {
 public:
     //! Life cycle
    NeuroEvo() {}
    ~NeuroEvo(void) { deletePopulation(); }
    void deep_copy(const NeuroEvo &NE);
    void deletePopulation();

    //! Class variables
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
        return (x->getEvaluation() > y->getEvaluation());
    }
    Action get_action(State state);
    Action get_action(std::vector<State> state);
    void save(std::string fileout);

 private:
    size_t  k_population_size_;
};
#endif  // SRC_LEARNING_INCLUDE_NEUROEVO_H_
