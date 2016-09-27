// Copyright 2016 Carrie Rebhuhn
#ifndef SRC_DOMAINS_UTM_FIX_H_
#define SRC_DOMAINS_UTM_FIX_H_

// STL includes
#include <list>
#include <vector>
#include <utility>

// Library includes

#include "Planning/include/LinkGraph.h"
#include "UAV.h"

class Fix {
 public:
    typedef std::pair<size_t, size_t> edge;
    Fix(easymath::XY loc, size_t id, LinkGraph* high_graph,
        std::vector<easymath::XY> dest_locs);


    virtual ~Fix() {}
    virtual UAV* generateUav(size_t step);
    void reset() { generateUav(true); }
    virtual UAV* generateUav(bool reset = false);

 protected:
    bool shouldGenerateUav(size_t step);
public:
    void resetUav(UAV* u) {
        auto e = high_graph_->get_locations();
        size_t index;
        do {
            index = rand() % e.size();
        } while (index == k_id_);

        u->reset(k_id_, index);
    }
    // A pointer to a list of UAVs that have arrived at the fix as their destination ~B
    std::list<UAV*> * uavs_stationed_;
    size_t k_id_, k_gen_rate_;
    easymath::XY k_loc_;
    LinkGraph* high_graph_;
    std::string k_traffic_mode_, k_destination_mode_;
    double k_gen_prob_;
    std::vector<easymath::XY> k_destination_locs_;
};
#endif  // SRC_DOMAINS_UTM_FIX_H_

