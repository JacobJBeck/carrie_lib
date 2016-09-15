// Copyright 2016 Carrie Rebhuhn
#ifndef SRC_LEARNING_INCLUDE_NEURALNET_H_
#define SRC_LEARNING_INCLUDE_NEURALNET_H_

#include <vector>
#include <iostream>
#include <random>
#include <string>
#include <algorithm>

#include "Math/include/easymath.h"
#include "FileIO/include/FileIn.h"
#include "FileIO/include/FileOut.h"
#include "IPolicy.h"

typedef matrix1d State;
typedef matrix1d Action;
typedef double Reward;
class NeuralNet : public IPolicy<State, Action, Reward> {
 public:
    typedef Action Action;
    typedef State State;
    typedef Reward Reward;
    // Life cycle
    NeuralNet(size_t num_input, size_t num_hidden, size_t num_output,
        double gamma = 0.9);
    NeuralNet() : gamma_(0.9) {}
    virtual ~NeuralNet() {}

    // Mutators
    void update(Reward R) { evaluation_ = R; }
    void mutate();  // different if child class
    void load(std::string file_in);
    void load(matrix1d node_info, matrix1d wt_info);

    // Accessors
    Action operator()(State s) { return predictContinuous(s); }
    Reward getEvaluation() const { return evaluation_; }
    void save(std::string fileout);

 private:
    struct Layer {
        size_t num_nodes_above_, num_nodes_below_;
        matrix2d w_bar_, w_;
        Layer(size_t above, size_t below);
    };
    std::vector<Layer> layers_;
    matrix1d predictContinuous(const matrix1d o);

    double evaluation_;
    double gamma_;
    double mut_std_;          //! mutation standard deviation
    double mut_rate_;        //! probability that each connection is changed
    matrix2d mult_buffer_;  //! container for all matrix multiplication output

    //! sets storage for matrix multiplication.
    //! Must be called each time network structure is changed/initiated
    //! sets    mult_buffer_[0].size() = [HIDDEN]
    //!         mult_buffer_[1].size() = [OUTPUT]
    void reserveMultBuffer();
    matrix1d getTopology();


    //! Static functions
    static void matrixMultiply(const matrix1d &A, const matrix2d &B,
        matrix1d *C);
    static matrix2d matrixMultiply(const matrix2d &A, const matrix2d &B);
    static matrix2d matrixMultiply(const matrix1d&A, const matrix1d &B);
    static matrix1d matrixMultiply(const matrix2d &A, const matrix1d &B);
    static matrix1d matrixMultiply(const matrix1d &A, const matrix2d &B);
    static void sigmoid(matrix1d *myVector);
    static void cmp_int_fatal(size_t a, size_t b);
    static double randSetFanIn(double fan_in);
    static double randAddFanIn(double fan_in, double mut_rate, double mut_std);
    static matrix1d flatten(const matrix2d &A);
};
#endif  // SRC_LEARNING_INCLUDE_NEURALNET_H_
