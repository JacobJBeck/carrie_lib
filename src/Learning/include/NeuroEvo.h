// Copyright 2016 Carrie Rebhuhn
#ifndef SRC_LEARNING_INCLUDE_NEUROEVO_H_
#define SRC_LEARNING_INCLUDE_NEUROEVO_H_

#include <string>
#include <vector>

#include "NeuralNet.h"
#include "Evolution.h"
#include "FileIO/include/FileIn.h"

class NeuroEvo : public Evolution<NeuralNet> {
 public:
    //! Life cycle
    NeuroEvo(size_t num_input, size_t num_hidden,
        size_t num_output, size_t population_size=10);
    ~NeuroEvo(void) { deletePopulation(); }
    void deepCopy(const NeuroEvo &NE);
    void deletePopulation();

    //! Mutators
    void generateNewMembers();
    bool selectNewMember();
    void selectSurvivors();
    void updatePolicyValues(double R);
    void load(std::string filein);

    //! Accessors
    double getBestMemberVal();
    static bool NNCompare(const NeuralNet *x, const NeuralNet *y) {
        return (x->getEvaluation() > y->getEvaluation());
    }
    Action getAction(State state);
    Action getAction(std::vector<State> state);
    void save(std::string fileout);

 private:
    size_t k_population_size_;
};
#endif  // SRC_LEARNING_INCLUDE_NEUROEVO_H_
