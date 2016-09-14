// Copyright 2016 Carrie Rebhuhn
#ifndef SRC_LEARNING_INCLUDE_NEURALNET_H_
#define SRC_LEARNING_INCLUDE_NEURALNET_H_

#include <vector>
#include <iostream>
#include <random>
#include <string>

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
    NeuralNet() : gamma_(0.9) {};
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
        Layer(size_t above, size_t below) :
            num_nodes_above_(above), num_nodes_below_(below) {
            // Populate Wbar with small random weights, including bias
            w_bar_ = easymath::zeros(above + 1, below);

            for (matrix1d &wt_outer : w_bar_) {
                for (double & wt_inner : wt_outer) {
                    double fan_in = above + 1.0;
                    wt_inner = randSetFanIn(fan_in);
                }
            }
            w_ = w_bar_;
            w_.pop_back();
        }
    };
    std::vector<Layer> layers_;
    matrix1d predictContinuous(const matrix1d o);

    double evaluation_;
    double gamma_;
    double mutStd;  // mutation standard deviation
    double mutRate_;  // probability that each connection is changed

    //! container for all outputs on way through neural network:
    //! for FAST multiplication
    matrix2d matrix_multiplication_storage;

    //! sets storage for matrix multiplication.
    //! Must be called each time network structure is changed/initiated
    void setMatrixMultiplicationStorage();


    double randAddFanIn(double fan_in);

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
};
#endif  // SRC_LEARNING_INCLUDE_NEURALNET_H_
