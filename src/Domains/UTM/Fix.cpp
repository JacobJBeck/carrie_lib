// Copyright 2016 Carrie Rebhuhn
#include "Fix.h"
#include <vector>
#include <list>
#include <map>
#include <string>
#include "yaml-cpp/yaml.h"

using std::vector;
using std::list;
using std::map;
using easymath::XY;
using easymath::rand;

Fix::Fix(XY loc, size_t id, LinkGraph* high_graph,
    vector<XY> dest_locs) :
    high_graph_(high_graph), k_destination_locs_(dest_locs), k_id_(id),
    k_loc_(loc) {
    YAML::Node config = YAML::LoadFile("config.yaml");
    k_traffic_mode_ = config["modes"]["traffic"].as<std::string>();
    k_destination_mode_ = config["modes"]["destinations"].as<std::string>();
    if (k_traffic_mode_ == "probabilistic")
        k_gen_prob_ = config["constants"]["generation_probability"].as<double>();
    else if (k_traffic_mode_ == "generated")
        k_gen_rate_ = config["constants"]["generation_rate"].as<size_t>();
}

bool Fix::shouldGenerateUav(size_t step) {
    if (k_traffic_mode_ == "constant") {
        return false;
    } else if (k_traffic_mode_ == "probabilistic") {
        double pnum = rand(0, 1);
        if (pnum > k_gen_prob_)
            return false;
        else
            return true;
    } else {
        // deterministic
        if (step% k_gen_rate_ != 0)
            return false;
        else
            return true;
    }
}

UAV* Fix::generateUav(size_t step) {
    // Creates a new UAV in the world
    if (shouldGenerateUav(step))
        return generateUav();
    else
        return NULL;
}

UAV* Fix::generateUav(bool reset) {
    static int calls = 0;
    auto e = high_graph_->get_edges();
    XY end_loc;
    if (k_destination_mode_ == "static") {
        size_t index = calls%e.size();
        if (e[index].first == k_id_) {
            end_loc = k_destination_locs_[e[index].second];
        } else {
            end_loc = k_destination_locs_[e[index].first];
        }
    } else {
        do {
            size_t index = rand() % e.size();
            end_loc = k_destination_locs_[e[index].second];
        } while (end_loc == k_loc_);
    }
    calls++;

    UAV* u = new UAV(high_graph_->get_membership(k_loc_),
        high_graph_->get_membership(end_loc),
        high_graph_);
    return u;
}
