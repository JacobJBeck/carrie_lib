// Copyright 2016 Carrie Rebhuhn
#ifndef SRC_DOMAINS_IREWARD_H_
#define SRC_DOMAINS_IREWARD_H_

#include <string>

class IReward {
 protected:
    IReward() {}
    std::string name;
};

class DiffAvg: public IReward {
    // The difference reward for the 'average'
    DiffAvg() {
        name = "DiffAvg";
    }
};

class DiffDownstream: public IReward {
    DiffDownstream() {
        name = "DiffDownstream";
    }
};

class DiffRealloc : public IReward {
    DiffRealloc() {
        name = "DiffRealloc";
    }
};

class DiffTouched : public IReward {
    DiffTouched() {
        name = "DiffTouched";
    }
};

class DiffNeuralNet : public IReward {
    DiffNeuralNet() {
        name = "DiffNeuralNet";
        size_t k_training_samples = 500000; // number of samples needed to approximate reward
        // TODO: deep net? Convolutional net? Can we make this episodic?
    }
};

class Global: public IReward {
    Global() {
        name = "Global";
    }
};

#endif  // SRC_DOMAINS_IREWARD_H_
