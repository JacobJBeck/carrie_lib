// Copyright 2016 Carrie Rebhuhn
#ifndef SRC_LEARNING_INCLUDE_NEURALNET_H_
#define SRC_LEARNING_INCLUDE_NEURALNET_H_

#include <vector>
#include <string>
#include <iostream>
#include "Math/include/easymath.h"
#include "FileIO/include/FileIn.h"
#include "IPolicy.h"

matrix2d operator*(const matrix2d &A, const matrix2d &B);
matrix1d operator*(const matrix2d &A, const matrix1d &B);
matrix1d operator*(const matrix1d &A, const matrix2d &B);
matrix2d operator*(const matrix1d &A, const matrix1d &B);

matrix1d operator*(const double A, matrix1d B);
matrix2d operator*(const double A, matrix2d B);
matrix1d operator-(matrix1d A, const matrix1d &B);
matrix2d operator-(matrix2d A, const matrix2d &B);
matrix1d dot(matrix1d m1, const matrix1d &m2);

typedef matrix1d State;
typedef matrix1d Action;
typedef double Reward;

class NeuralNet : public IPolicy<State, Action, Reward> {
 public:
    typedef Action Action;
    typedef State State;
    typedef Reward Reward;
    // Life cycle

    NeuralNet() : k_gamma_(0.9), k_eta_(0.1), N(2) {}
    NeuralNet(size_t num_input, size_t num_hidden, size_t num_output,
        double gamma = 0.9, double eta = 0.1);
    virtual ~NeuralNet() {}

    size_t N;
    matrix2d s, z, f, d;

    matrix1d sigmoidDerivative(matrix1d X) {
        matrix1d one = (matrix1d(X.size(), 1.0));
        return dot(sigmoid(X), one - sigmoid(X));
    }
    matrix2d T(matrix2d m);

    matrix1d fwrdProp(matrix1d X) {
        X.push_back(1.0);
        s[0] = X*layers_[0].w_bar_;
        z[0] = sigmoid(s[0]);
        f[0] = sigmoidDerivative(s[0]);

        for (size_t i = 1; i < N; i++) {
            auto zbias = z[i-1];
            zbias.push_back(1.0);
            s[i] = zbias*layers_[i].w_bar_;
            z[i] = sigmoid(s[i]);
            f[i] = sigmoidDerivative(s[i]);
        }
        return z[N - 1];
    }

    void backProp(matrix1d Y) {
        d[N-1] = z[N-1] - Y;

        int startnum = N - 2;
        int endnum = 0;

        //for (size_t i = startnum; i >= endnum; i--) {
        int i = 0;
            auto w_nobias = layers_[i].w_;
            matrix2d m = w_nobias[i] * d[i + 1];
            
            d[i] = dot(f[i], flatten(m));
        //}
    }

    void updateWt(matrix1d X) {
        X.push_back(1.0);
        layers_[0].w_bar_ = layers_[0].w_bar_ - k_eta_*T(d[0]*X);
        for (size_t i = 0, j = 1; j < N; i++, j++) {
            matrix1d zbias = z[i];
            zbias.push_back(1.0);
            matrix2d mod = T(d[j] * zbias);
            layers_[j].w_bar_ = layers_[j].w_bar_ - k_eta_*(mod);
        }
    }

    // Mutators
    void update(Reward R) { evaluation_ = R; }
    void mutate();  // different if child class
    void load(std::string file_in);
    void load(matrix1d node_info, matrix1d wt_info);
    double backProp(const matrix1d &X, const matrix1d &Y);


    // Accessors
    Action operator()(State s) {
        return fwrdProp(s);
    }
    Reward getEvaluation() const { return evaluation_; }
    void save(std::string fileout);

    static void cmpIntFatal(size_t a, size_t b);

 private:
    struct Layer {
        size_t k_num_nodes_above_, k_num_nodes_below_;
        matrix2d w_bar_, w_;
        Layer(size_t above, size_t below);
    };
    std::vector<Layer> layers_;

    double evaluation_;
    double k_gamma_, k_eta_;
    double k_mut_std_;      //! mutation standard deviation
    double k_mut_rate_;     //! probability that each connection is changed
    matrix2d mult_buffer_;  //! container for all matrix multiplication output

    //! sets storage for matrix multiplication.
    //! Must be called each time network structure is changed/initiated
    //! sets    mult_buffer_[0].size() = [HIDDEN]
    //!         mult_buffer_[1].size() = [OUTPUT]
    void reserveMultBuffer();
    matrix1d getTopology();

    static void sigmoid(matrix1d *myVector);
    static matrix1d sigmoid(matrix1d vec);
    static double randSetFanIn(double fan_in);
    static double randAddFanIn(double fan_in, double mut_rate, double mut_std);
    static matrix1d flatten(const matrix2d &A);
};
#endif  // SRC_LEARNING_INCLUDE_NEURALNET_H_
