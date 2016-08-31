// Copyright 2016 Carrie Rebhuhn
#ifndef DOMAINS_UTM_FIX_H_
#define DOMAINS_UTM_FIX_H_

// STL includes
#include <list>
#include <vector>
#include <utility>

// Library includes
#include "Planning/MultiGraph.h"
#include "UAV.h"

class Fix {
 public:
    typedef std::pair<size_t, size_t> edge;
    Fix(easymath::XY loc, size_t ID, MultiGraph<LinkGraph>* highGraph,
        std::vector<easymath::XY> dest_locs, size_t n_types_set);


    virtual ~Fix() {}
    virtual UAV* generate_UAV(size_t step);

    void reset() {
        generate_UAV(true);
    }

    virtual UAV* generate_UAV(bool reset = false);

 protected:
    bool should_generate_UAV(size_t step);

    // A pointer to a list of UAVs that have arrived at the fix as their destination ~B
    std::list<UAV*> * UAVs_stationed;
    size_t ID, genrate, n_types, n_uavs;
    easymath::XY loc;
    MultiGraph<LinkGraph>* highGraph;
    std::string traffic_mode;
    double pgen;
    std::vector<easymath::XY> destination_locs;
};
#endif  // DOMAINS_UTM_FIX_H_

