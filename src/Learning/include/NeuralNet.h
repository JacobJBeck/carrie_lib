// Copyright 2016 Carrie Rebhuhn
#ifndef SRC_LEARNING_NEURALNET_H_
#define SRC_LEARNING_NEURALNET_H_

#include <vector>
#include <iostream>
#include <chrono>
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
//    NeuralNet() : evaluation_(0.0), gamma_(0.9) {}
    NeuralNet(size_t num_input, size_t num_hidden,
        size_t num_output, double gamma = 0.9);
    explicit NeuralNet(std::vector<size_t> &, double gamma = 0.9);
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
    void save(matrix1d *node_info, matrix1d *wt_info);

protected:
    struct Layer {
        size_t num_nodes_above_, num_nodes_below_;
        matrix2d w_bar_, w_;
        Layer(size_t above, size_t below) :
            num_nodes_above_(above), num_nodes_below_(below) {
            // Populate Wbar with small random weights, including bias
            w_bar_ = easymath::zeros(above + 1, below);

            for (std::vector<double>& wt_outer : w_bar_) {
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

    void addInputs(int nToAdd);

    void train(const matrix2d &O, const matrix2d &T, double epsilon = 0.0,
        int iterations = 0);
    matrix1d predictBinary(const matrix1d o);
    matrix1d predictContinuous(const matrix1d o);
    matrix2d batchPredictBinary(const matrix2d &O);
    matrix2d batchPredictContinuous(const matrix2d &O);


    double evaluation_;
    double gamma_;
    double mutStd;  // mutation standard deviation
    double mutRate_;  // probability that each connection is changed

    //! container for all outputs on way through neural network:
    //! for FAST multiplication
    matrix2d matrix_multiplication_storage;
    //! number of nodes at each layer of the network
    std::vector<size_t> nodes_;

    //! weights, W[interface][input][hidden(next unit)]. Without bias
    matrix3d W;
    //! weights with bias;
    matrix3d Wbar;

    //! sets weights randomly for the defined network
    void setRandomWeights();

    //! sets storage for matrix multiplication.
    //! Must be called each time network structure is changed/initiated
    void setMatrixMultiplicationStorage();

    //size_t connections();
    double backProp(const matrix1d &o, const matrix1d &t);
    void feedForward(const matrix1d &o, matrix2d* Ohat, matrix3d* D);


    //! Static functions
    static double SSE(const matrix1d &myVector);
    static void matrixMultiply(const matrix1d &A, const matrix2d &B,
        matrix1d *C);
    static matrix2d matrixMultiply(const matrix2d &A, const matrix2d &B);
    static matrix2d matrixMultiply(const matrix1d&A, const matrix1d &B);
    static matrix1d matrixMultiply(const matrix2d &A, const matrix1d &B);
    static matrix1d matrixMultiply(const matrix1d &A, const matrix2d &B);
    static void sigmoid(matrix1d *myVector);
    static void cmp_int_fatal(size_t a, size_t b);
    double randAddFanIn(double fan_in);
    double static randSetFanIn(double fan_in);
};
#endif  // SRC_LEARNING_NEURALNET_H_
