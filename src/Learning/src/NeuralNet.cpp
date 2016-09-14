// Copyright 2016 Carrie Rebhuhns
#include "NeuralNet.h"
#include <vector>
#include <string>
#include <ctime>

using easymath::rand;
using easymath::sum;
using std::vector;
using std::string;

double NeuralNet::randSetFanIn(double fan_in) {
    // For initialization of the neural net weights
    double rand_neg1to1 = rand(-1, 1)*0.1;
    double scale_factor = 100.0;
    return scale_factor*rand_neg1to1 / sqrt(fan_in);
}

void NeuralNet::mutate() {
    for (Layer &l : layers_) {
        for (matrix1d &wt_outer : l.w_bar_) {
            // #pragma parallel omp for
            for (double &w : wt_outer) {
                double fan_in = static_cast<double>(l.num_nodes_above_);
                w += randAddFanIn(fan_in);
            }
        }
    }
}

double NeuralNet::randAddFanIn(double fan_in) {
    // Adds random amount mutRate_% of the time,
    // amount based on fan_in and mutstd
    if (rand(0, 1) > mutRate_) {
        return 0.0;
    } else {
        // FOR MUTATION
        std::default_random_engine generator;
        generator.seed(static_cast<size_t>(time(NULL)));
        std::normal_distribution<double> distribution(0.0, mutStd);
        return distribution(generator);
    }
}

NeuralNet::NeuralNet(size_t num_inputs, size_t num_hidden, size_t num_outputs,
    double gamma): gamma_(gamma),
    evaluation_(0), mutRate_(0.5), mutStd(1.0) {
    layers_.push_back(Layer(num_inputs, num_hidden));
    layers_.push_back(Layer(num_hidden, num_outputs));

    setMatrixMultiplicationStorage();
}

void NeuralNet::load(string filein) {
    matrix2d wts = easyio::read2<double>(filein);
    load(wts[0], wts[1]);
}

void NeuralNet::save(string fileout) {
    matrix2d outmatrix(2);
    outmatrix[0].push_back(layers_[0].num_nodes_above_);
    for (Layer l : layers_) {
        outmatrix[0].push_back(static_cast<double>(l.num_nodes_below_));
        for (auto wt_outer : l.w_bar_) {
            outmatrix[1].insert(outmatrix[1].end(), wt_outer.begin(),
                wt_outer.end());
        }
    }
    FileOut::print_vector(outmatrix, fileout);
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
    setMatrixMultiplicationStorage();
}

void NeuralNet::setMatrixMultiplicationStorage() {
    matrix_multiplication_storage = matrix2d();
    for (auto l : layers_)
        matrix_multiplication_storage.push_back(matrix1d(l.num_nodes_above_));
    matrix_multiplication_storage.push_back
        (matrix1d(layers_.back().num_nodes_below_));
}

matrix1d NeuralNet::predictContinuous(matrix1d observations) {
    observations.push_back(1.0);
    matrixMultiply(observations, layers_[0].w_bar_,
        &matrix_multiplication_storage[0]);
    sigmoid(&matrix_multiplication_storage[0]);

    for (size_t i = 1; i < layers_.size(); i++) {
        matrix_multiplication_storage[i - 1].back() = 1.0;
        matrixMultiply(matrix_multiplication_storage[i - 1],
            layers_[i].w_bar_, &matrix_multiplication_storage[i]);
        sigmoid(&matrix_multiplication_storage[i]);
    }
    return matrix_multiplication_storage.back();
}

matrix2d NeuralNet::matrixMultiply(const matrix2d &A, const matrix2d &B) {
    // returns a size(A,1)xsize(B,2) matrix
    // printf("mm");
    cmp_int_fatal(A[0].size(), B.size());

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

matrix2d NeuralNet::matrixMultiply(const matrix1d &A, const matrix1d &B) {
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

matrix1d NeuralNet::matrixMultiply(const matrix2d &A, const matrix1d &B) {
    // returns a size(A,1)x1 matrix
    // assumes B is a COLUMN vector
    // printf("mm1");
    cmp_int_fatal(A[0].size(), B.size());

    matrix1d C(A.size(), 0.0);
    for (size_t row = 0; row < A.size(); row++) {
        for (size_t inner = 0; inner < B.size(); inner++) {
            C[row] += A[row][inner] * B[inner];
        }
    }

    return C;
}

matrix1d NeuralNet::matrixMultiply(const matrix1d &A, const matrix2d &B) {
    // Use this if expecting to get a vector back;
    // assumes A is a ROW vector (1xcols)
    // returns a 1xsize(B,2) matrix

    cmp_int_fatal(A.size(), B.size());

    // MODIFY TO MATCH1
    matrix1d C(B[0].size(), 0.0);
    for (size_t col = 0; col < B[0].size(); col++) {
        for (size_t inner = 0; inner < B.size(); inner++) {
            C[col] += A[inner] * B[inner][col];
        }
    }

    return C;
}

void NeuralNet::matrixMultiply(const matrix1d &A, const matrix2d &B,
    matrix1d* C) {
    /* This fills C up to the size of B[0].size().
    * C is allowed to be larger by 1, to accommodate bias
    * Use this if expecting to get a vector back;
    * assumes A is a ROW vector (1xcols)
    * returns a 1xsize(B,2) matrix*/

    cmp_int_fatal(A.size(), B.size());
    if (B[0].size() != C->size() && B[0].size() != C->size() - 1) {
        printf("B and C sizes don't match. pausing");
        system("pause");
    }

    // MODIFY TO MATCH1
    for (size_t col = 0; col < B[0].size(); col++) {
        C->at(col) = 0.0;
        for (size_t inner = 0; inner < B.size(); inner++) {
            C->at(col) += A[inner] * B[inner][col];
        }
    }
}

void NeuralNet::sigmoid(matrix1d *myVector) {
    for (size_t i = 0; i < myVector->size(); i++) {
        myVector->at(i) = 1 / (1 + exp(-myVector->at(i)));
    }
}

void NeuralNet::cmp_int_fatal(size_t a, size_t b) {
    if (a != b) {
        printf("Sizes do not match! Pausing to debug then exiting.");
        system("pause");
        exit(1);
    }
}
