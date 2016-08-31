// Copyright 2016 Carrie Rebhuhn
#include "NeuralNet.h"
#include <vector>
#include <string>

using easymath::rand;
using easymath::sum;
using std::vector;
using std::string;

double NeuralNet::randAddFanIn(double) {
    // Adds random amount mutationRate% of the time,
    // amount based on fan_in and mutstd
    if (rand(0, 1) > mutationRate) {
        return 0.0;
    } else {
        // FOR MUTATION
        std::default_random_engine generator;
        generator.seed(static_cast<size_t>(time(NULL)));
        std::normal_distribution<double> distribution(0.0, mutStd);
        return distribution(generator);
    }
}

double NeuralNet::randSetFanIn(double fan_in) {
    // For initialization of the neural net weights
    double rand_neg1to1 = rand(-1, 1)*0.1;
    double scale_factor = 100.0;
    return scale_factor*rand_neg1to1 / sqrt(fan_in);
}

void NeuralNet::mutate() {
    for (size_t i = 0; i < Wbar.size(); i++) {
        for (size_t j = 0; j < Wbar[i].size(); j++) {
            // #pragma parallel omp for
            for (size_t k = 0; k < Wbar[i][j].size(); k++) {
                double fan_in = static_cast<double>(Wbar[i].size());
                Wbar[i][j][k] += randAddFanIn(fan_in);
            }
        }
    }
}

void NeuralNet::setRandomWeights() {
    Wbar = matrix3d(connections());
    W = matrix3d(connections());
    for (int c = 0; c < connections(); c++) {  // number of layers
        int above = c;
        int below = c + 1;

        // Populate Wbar with small random weights, including bias
        Wbar[c] = (matrix2d(nodes_[above] + 1));

        // above+1 include bias
        for (int i = 0; i < nodes_[above] + 1; i++) {
            // reserve memory for the connections below
            Wbar[c][i] = matrix1d(nodes_[below]);
            for (int j = 0; j < nodes_[below]; j++) {
                double fan_in = nodes_[above] + 1.0;
                Wbar[c][i][j] = randSetFanIn(fan_in);
            }
        }

        W[c] = Wbar[c];
        W[c].pop_back();  // remove extra bias weights
    }
}

NeuralNet::NeuralNet(size_t nInputs, size_t nHidden, size_t nOutputs, double
    gamma) :nodes_(vector<size_t>(3)), gamma_(gamma),
    evaluation(0), mutationRate(0.5), mutStd(1.0) {
    nodes_[0] = nInputs;
    nodes_[1] = nHidden;
    nodes_[2] = nOutputs;

    setRandomWeights();
    setMatrixMultiplicationStorage();
}

void NeuralNet::load(string filein) {
    // loads neural net specs
    matrix2d wts = easyio::read2<double>(filein);

    // CURRENTLY HARDCODED TO ONLY ALLOW A SINGLE LAYER

    /// TOP CONTAINS TOPOLOGY INFORMATION
    nodes_ = vector<size_t>(3);
    nodes_[0] = static_cast<int>(wts[0][0]);
    nodes_[1] = static_cast<int>(wts[0][1]);
    nodes_[2] = static_cast<int>(wts[0][2]);

    Wbar = matrix3d(connections());
    W = matrix3d(connections());
    int index = 0;  // index for accessing NN elements
    for (int c = 0; c < connections(); c++) {  // number of layers
        int above = c;
        int below = c + 1;

        // Populate Wbar with loaded weights, including bias
        Wbar[c] = (matrix2d(nodes_[above] + 1));
        for (int i = 0; i < nodes_[above] + 1; i++) {
            // reserve memory for the connections below
            Wbar[c][i] = matrix1d(nodes_[below]);
            for (int j = 0; j < nodes_[below]; j++) {
                Wbar[c][i][j] = wts[1][index++];
            }
        }

        W[c] = Wbar[c];
        W[c].pop_back();  // remove extra bias weights
    }
    setMatrixMultiplicationStorage();
}

void NeuralNet::save(string fileout) {
    matrix2d outmatrix(2);
    for (size_t i = 0; i < nodes_.size(); i++) {
        outmatrix[0].push_back(static_cast<double>(nodes_[i]));
    }

    for (int c = 0; c < connections(); c++) {
        int above = c;
        int below = c + 1;

        for (int i = 0; i < nodes_[above] + 1; i++) {  // above+1 include bias
            for (int j = 0; j < nodes_[below]; j++) {
                outmatrix[1].push_back(Wbar[c][i][j]);
            }
        }
    }
    FileOut::print_vector(outmatrix, fileout);
}

void NeuralNet::load(matrix1d node_info, matrix1d wt_info) {
    // CURRENTLY HARDCODED TO ONLY ALLOW A SINGLE LAYER

    /// TOP CONTAINS TOPOLOGY INFORMATION
    nodes_ = vector<size_t>(3);
    nodes_[0] = static_cast<int>(node_info[0]);
    nodes_[1] = static_cast<int>(node_info[1]);
    nodes_[2] = static_cast<int>(node_info[2]);

    Wbar = matrix3d(connections());
    W = matrix3d(connections());
    int index = 0;  // index for accessing NN elements

    // Connections are number of layers
    for (int c = 0; c < connections(); c++) {
        int above = c;
        int below = c + 1;

        // Populate Wbar with loaded weights, including bias
        Wbar[c] = (matrix2d(nodes_[above] + 1));
        // above+1 include bias;
        for (int i = 0; i < nodes_[above] + 1; i++) {
            // reserve memory for the connections below
            Wbar[c][i] = matrix1d(nodes_[below]);
            for (int j = 0; j < nodes_[below]; j++) {
                Wbar[c][i][j] = wt_info[index++];
            }
        }

        W[c] = Wbar[c];
        W[c].pop_back();  // remove extra bias weights
    }
    setMatrixMultiplicationStorage();
}



void NeuralNet::save(matrix1d *node_info, matrix1d *wt_info) {
    *node_info = matrix1d(nodes_.size());

    for (size_t i = 0; i < nodes_.size(); i++) {
        node_info->at(i) = static_cast<double>(nodes_[i]);
    }

    for (int c = 0; c < connections(); c++) {  // number of layers
        int above = c;
        int below = c + 1;

        for (int i = 0; i < nodes_[above] + 1; i++) {  // above+1 include bias;
            for (int j = 0; j < nodes_[below]; j++) {
                wt_info->push_back(Wbar[c][i][j]);
            }
        }
    }
}

void NeuralNet::setMatrixMultiplicationStorage() {
    // Allocates space for the matrix multiplication storage container,
    // based on current Wbar/connections()
    matrix_multiplication_storage = matrix2d(connections());
    for (int connection = 0; connection < connections(); connection++) {
        matrix_multiplication_storage[connection]
            = matrix1d(Wbar[connection][0].size(), 0.0);
        if (connection + 1 != connections()) {  // if not the output layer
            matrix_multiplication_storage[connection].push_back(1.0);
        }
    }
}

void NeuralNet::addInputs(int nToAdd) {
    nodes_[0] += nToAdd;

    // add new connections leading to each of the lower nodes
    for (int i = 0; i < nToAdd; i++) {
        // adding another connection, each new one leads to nodes below
        Wbar[0].push_back(matrix1d(nodes_[1], 0.0));

        /*for (int j=0; j<nodes_[1]; j++){
            double fan_in = nodes_[0]+1.0;
            double rand_neg1to1 = (double(rand())/double(RAND_MAX))*2.0-1.0;
            Wbar[0].back()[j]=rand_neg1to1/sqrt(fan_in);
            Wbar[0].back()[j]=0.0;
        }*/
    }
    W[0] = Wbar[0];
    W[0].pop_back();

    setMatrixMultiplicationStorage();
}

NeuralNet::NeuralNet(vector<size_t> &nodes, double gamma) :
    evaluation(0.0), nodes_(nodes), gamma_(gamma) {
    setRandomWeights();
    setMatrixMultiplicationStorage();
}

void NeuralNet::train(const matrix2d &observations, const matrix2d &T,
    double epsilon, int iterations) {
    // just ensure it's bigger always to begin...
    double err = 2 * epsilon + 1.0;

    if (iterations == 0) {
        while (err >= epsilon) {
            matrix1d errs;
            for (size_t i = 0; i < observations.size(); i++) {
                errs.push_back(backProp(observations[i], T[i]));
            }
            err = sum(errs);
            printf("Err=%f\n", err);
        }
    } else {
        int step = 0;

        while (err >= epsilon && iterations >= step) {
            matrix1d errs;
            for (size_t i = 0; i < observations.size(); i++) {
                errs.push_back(backProp(observations[i], T[i]));
            }
            err = sum(errs);
            printf("Err=%f\n", err);
            step++;
        }
    }
}

matrix1d NeuralNet::predictBinary(matrix1d observations) {
    for (int connection = 0; connection < connections(); connection++) {
        observations.push_back(1.0);  // add 1 for bias
        observations = matrixMultiply(observations, Wbar[connection]);
        sigmoid(&observations);  // Compute outputs
    }
    return observations;
}

matrix1d NeuralNet::predictContinuous(matrix1d observations) {
    observations.push_back(1.0);
    matrixMultiply(observations, Wbar[0], &matrix_multiplication_storage[0]);
    sigmoid(&matrix_multiplication_storage[0]);

    for (int connection = 1; connection < connections(); connection++) {
        // static size allocation.
        // last element is set to 1.0, bias (may not need to?)
        matrix_multiplication_storage[connection - 1].back() = 1.0;

        matrixMultiply(matrix_multiplication_storage[connection - 1],
            Wbar[connection], &matrix_multiplication_storage[connection]);
        sigmoid(&matrix_multiplication_storage[connection]);
    }

    return matrix_multiplication_storage.back();
}

matrix2d NeuralNet::batchPredictBinary(const matrix2d &observations) {
    matrix2d out;
    for (size_t i = 0; i < observations.size(); i++) {
        out.push_back(predictBinary(observations[i]));
    }
    return out;
}

matrix2d NeuralNet::batchPredictContinuous(const matrix2d &observations) {
    matrix2d out;
    for (size_t i = 0; i < observations.size(); i++) {
        out.push_back(predictContinuous(observations[i]));
    }
    return out;
}

double NeuralNet::SSE(const matrix1d &myVector) {
    double err = 0.0;
    for (size_t i = 0; i < myVector.size(); i++) {
        err += myVector[i] * myVector[i];
    }
    return err;
}

size_t NeuralNet::connections() {
    return nodes_.size() - 1;
}

double NeuralNet::backProp(const matrix1d &observations, const matrix1d &t) {
    // 'observations' is the input vector, 't' is the 'target vector'
    // returns the SSE for the output vector

    matrix2d Ohat;  // outputs with bias
    matrix3d D;  // stored derivatives
    // Go through network "feed forward" computation

    feedForward(observations, &Ohat, &D);

    // "stored derivatives of the quadratic deviations"
    matrix1d e(Ohat.back().size() - 1, 0.0);

    for (size_t i = 0; i < Ohat.back().size() - 1; i++) {
        e[i] = (Ohat.back()[i] - t[i]);
    }

    // Hidden/output layer delta calcs
    matrix2d delta;
    for (int i = 0; i < connections(); i++) {
        delta.push_back(matrix1d());
    }
    delta.back() = matrixMultiply(D.back(), e);  // output layer delta

    // back propagation
    for (size_t connection = connections() - 2; connection >= 0; connection--) {
        matrix2d mult = matrixMultiply(D[connection], W[connection + 1]);
        delta[connection] = matrixMultiply(mult, delta[connection + 1]);
    }

    // Corrections to weights
    for (int c = 0; c < connections(); c++) {
        matrix2d DeltaWbarT =
            matrixMultiply(delta[c], Ohat[c]);
        for (size_t i = 0; i < Wbar[c].size(); i++) {
            for (size_t j = 0; j < Wbar[c][i].size(); j++) {
                // ji because it's transpose :)
                Wbar[c][i][j] -= gamma_*DeltaWbarT[j][i];
            }
        }

        W[c] = Wbar[c];
        W[c].pop_back();
    }

    // Calculate SS
    return SSE(e);
}

void NeuralNet::feedForward(const matrix1d &o, matrix2d *Ohat, matrix3d* D) {
    Ohat->push_back(o);
    Ohat->back().push_back(1.0);  // add 1 for bias

    for (int c = 0; c < connections(); c++) {
        Ohat->push_back(matrixMultiply(Ohat->at(c), Wbar[c]));
        sigmoid(&Ohat->back());

        // D stuff
        D->push_back(matrix2d());

        // number of hidden/output units, excluding bias
        size_t k = Ohat->back().size();

        for (int i = 0; i < k; i++) {  // add for last entry
            // For last output given, calculate the derivatives
            double Oi = Ohat->back().at(i);
            D->at(c).push_back(matrix1d(k, 0.0));
            D->at(c)[i][i] = Oi*(1 - Oi);  // create a diagonal matrix
        }

        Ohat->back().push_back(1.0);
    }
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
