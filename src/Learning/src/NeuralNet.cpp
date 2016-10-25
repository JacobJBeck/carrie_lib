// Copyright 2016 Carrie Rebhuhns
#include "NeuralNet.h"
#include <vector>
#include <string>
#include <ctime>
#include <random>

using easymath::rand;
using easymath::sum;
using std::vector;
using std::string;

matrix2d NeuralNet::T(matrix2d m) {
    matrix2d t(m[0].size(), matrix1d(m.size()));
    for (int i = 0; i < m.size(); i++) {
        for (int j = 0; j < m[0].size(); j++) {
            t[j][i] = m[i][j];
        }
    }
    return t;
}

double NeuralNet::backProp(const matrix1d &X, const matrix1d &Y) {
    // 'X' is the input vector, 'Y' is the 'target vector'
    // returns the SSE for the output vector

    fwrdProp(X);
    backProp(Y);
    updateWt(X);
    return 0.0;
}

NeuralNet::Layer::Layer(size_t above, size_t below) :
    k_num_nodes_above_(above), k_num_nodes_below_(below) {
    // Populate Wbar with small random weights, including bias
    w_bar_ = easymath::zeros(above + 1, below);

    for (matrix1d &w : w_bar_)
        std::generate(w.begin(), w.end(),
            std::bind(randSetFanIn, above + 1.0));
    w_ = w_bar_;
    w_.pop_back();
}

double NeuralNet::randSetFanIn(double fan_in) {
    return rand(-10, 10) / sqrt(fan_in);
}

void NeuralNet::mutate() {
    for (Layer &l : layers_) {
        for (matrix1d &wt_outer : l.w_bar_) {
            double fan_in = static_cast<double>(l.k_num_nodes_above_);
            // #pragma parallel omp for
            for (double &w : wt_outer) {
                w += randAddFanIn(fan_in, k_mut_rate_, k_mut_std_);
            }
        }
    }
}

double NeuralNet::randAddFanIn(double fan_in, double mut_rate, double mut_std) {
    // Adds random amount mutRate_% of the time,
    // amount based on fan_in and mut_std
    if (rand(0, 1) > mut_rate) {
        return 0.0;
    } else {
        std::default_random_engine generator;
        generator.seed(static_cast<size_t>(time(NULL)));
        std::normal_distribution<double> distribution(0.0, mut_std);
        // Note: divide by fan_in later
        return distribution(generator);
    }
}

NeuralNet::NeuralNet(size_t num_inputs, size_t num_hidden, size_t num_outputs,
    double gamma, double eta): k_gamma_(gamma), k_eta_(eta), evaluation_(0),
    k_mut_rate_(0.5), k_mut_std_(1.0), N(2) {
    
    vector<vector<double> > temp(2);

    s = temp;
    z = temp;
    f = temp;
    d = temp;

    // 1 hidden layer neural net (corresponds to N of 2)
    layers_.push_back(Layer(num_inputs, num_hidden));
    layers_.push_back(Layer(num_hidden, num_outputs));
    reserveMultBuffer();
}

void NeuralNet::load(string filein) {
    matrix2d wts = cio::read2<double>(filein);
    load(wts[0], wts[1]);
}

void NeuralNet::save(string fileout) {
    matrix2d out(2);
    out[0] = getTopology();
    for (Layer l : layers_) {
        matrix1d flat_wbar = flatten(l.w_bar_);
        out[1].insert(out[1].end(), flat_wbar.begin(), flat_wbar.end());
    }
    cio::print2(out, fileout);
}

void NeuralNet::load(matrix1d node_info, matrix1d wt_info) {
    size_t num_inputs = static_cast<int>(node_info[0]);
    size_t num_hidden = static_cast<int>(node_info[1]);
    size_t num_outputs = static_cast<int>(node_info[2]);

    layers_.push_back(Layer(num_inputs, num_hidden));
    layers_.push_back(Layer(num_hidden, num_outputs));

    int index = 0;  // index for accessing NN elements
    for (Layer &l : layers_) {  // number of layers
        for (matrix1d &wt_outer : l.w_bar_) {
            for (double &wt_inner : wt_outer) {
                wt_inner = wt_info[index++];
            }
        }
        l.w_ = l.w_bar_;
        l.w_.pop_back();
    }
    reserveMultBuffer();
}

void NeuralNet::reserveMultBuffer() {
    for (auto l : layers_)
        mult_buffer_.reserve(l.k_num_nodes_below_);
}

matrix1d NeuralNet::getTopology() {
    matrix1d topology(1);
    topology[0] = layers_.front().k_num_nodes_above_;
    for (Layer l : layers_)
        topology.push_back(static_cast<double>(l.k_num_nodes_below_));
    return topology;
}


matrix1d NeuralNet::sigmoid(matrix1d vec) {
    for (auto &v : vec) {
        v = 1 / (1 + exp(-v));
    }
    return vec;
}
void NeuralNet::sigmoid(matrix1d *myVector) {
    for (size_t i = 0; i < myVector->size(); i++) {
        myVector->at(i) = 1 / (1 + exp(-myVector->at(i)));
    }
}

void NeuralNet::cmpIntFatal(size_t a, size_t b) {
    if (a != b) {
        printf("Sizes do not match! Pausing to debug then exiting.");
        system("pause");
        exit(1);
    }
}

matrix1d NeuralNet::flatten(const matrix2d &A) {
    matrix1d B;
    for (auto a : A)
        B.insert(B.end(), a.begin(), a.end());
    return B;
}

matrix1d operator-(matrix1d A, const matrix1d &B) {
    for (size_t i = 0; i < A.size(); i++) {
        A[i] -= B[i];
    }
    return A;
}

matrix2d operator-(matrix2d A, const matrix2d &B) {
    for (size_t i = 0; i < A.size(); i++) {
        A[i] = A[i] - B[i];
    }
    return A;
}

matrix1d dot(matrix1d m1, const matrix1d &m2) {
    for (size_t i = 0; i < m1.size(); i++) {
        m1[i] *= m2[i];
    }
    return m1;
}

matrix2d operator*(const matrix2d &A, const matrix2d &B) {
    // returns a size(A,1)xsize(B,2) matrix
    // printf("mm");
    NeuralNet::cmpIntFatal(A[0].size(), B.size());

    matrix2d C(A.size());
    for (size_t row = 0; row < A.size(); row++) {
        C[row] = matrix1d(B[0].size(), 0.0);
        for (size_t col = 0; col < B[0].size(); col++) {
            for (size_t inner = 0; inner < B.size(); inner++) {
                C[row][col] += A[row][inner] * B[inner][col];
            }
        }
    }

    return C;
}

matrix2d operator*(const matrix1d &A, const matrix1d &B) {
    // returns a A.size()xB.size() matrix

    matrix2d C(A.size());
    for (size_t row = 0; row < A.size(); row++) {
        C[row] = matrix1d(B.size(), 0.0);
        for (size_t col = 0; col < B.size(); col++) {
            C[row][col] += A[row] * B[col];
        }
    }

    return C;
}

matrix1d operator*(const matrix2d &A, const matrix1d &B) {
    // returns a size(A,1)x1 matrix
    // assumes B is a COLUMN vector
    // printf("mm1");
    NeuralNet::cmpIntFatal(A[0].size(), B.size());

    matrix1d C(A.size(), 0.0);
    for (size_t row = 0; row < A.size(); row++) {
        for (size_t inner = 0; inner < B.size(); inner++) {
            C[row] += A[row][inner] * B[inner];
        }
    }

    return C;
}

matrix1d operator*(const matrix1d &A, const matrix2d &B) {
    // Use this if expecting to get a vector back;
    // assumes A is a ROW vector (1xcols)
    // returns a 1xsize(B,2) matrix

    NeuralNet::cmpIntFatal(A.size(), B.size());

    // MODIFY TO MATCH1
    matrix1d C(B[0].size(), 0.0);
    for (size_t col = 0; col < B[0].size(); col++) {
        for (size_t inner = 0; inner < B.size(); inner++) {
            C[col] += A[inner] * B[inner][col];
        }
    }

    return C;
}
