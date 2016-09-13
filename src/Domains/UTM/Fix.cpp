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

Fix::Fix(XY loc, size_t ID_set, MultiGraph<LinkGraph>* highGraph,
    vector<XY> dest_locs, size_t n_types_set) :
    highGraph(highGraph), destination_locs(dest_locs), ID(ID_set),
    loc(loc), n_types(n_types_set) {
    YAML::Node config = YAML::LoadFile("config.yaml");
    traffic_mode = config["modes"]["traffic"].as<std::string>();
    destination_mode = config["modes"]["destinations"].as<std::string>();
    if (traffic_mode == "probabilistic")
        pgen = config["constants"]["generation_probability"].as<double>();
    else if (traffic_mode == "generated")
        genrate = config["constants"]["generation_rate"].as<size_t>();
    else if (traffic_mode == "constant")
        n_uavs = config["constants"]["vehicles"].as<size_t>();
}

bool Fix::should_generate_UAV(size_t step) {
    if (traffic_mode == "constant") {
        return false;
    } else if (traffic_mode == "probabilistic") {
        double pnum = rand(0, 1);
        if (pnum > pgen)
            return false;
        else
            return true;
    } else {
        // deterministic
        if (step%genrate != 0)
            return false;
        else
            return true;
    }
}

UAV* Fix::generate_UAV(size_t step) {
    // Creates a new UAV in the world
    if (should_generate_UAV(step))
        return generate_UAV();
    else
        return NULL;
}

UAV* Fix::generate_UAV(bool reset) {
    static int calls = 0;
    if (reset)
        calls = 0;
    auto e = highGraph->at()->get_edges();
    XY end_loc;
    if (destination_mode == "static") {
        size_t index = calls%e.size();
        if (e[index].first == ID) {
            end_loc = destination_locs[e[index].second];
        } else {
            end_loc = destination_locs[e[index].first];
        }
    } else {
        do {
            size_t index = rand() % e.size();
            end_loc = destination_locs[e[index].second];
        } while (end_loc == loc);
    }
    
    // Creates an equal number of each type;
    int type_id_set = calls%n_types;
    calls++;

    UAV* u = new UAV(highGraph->at(type_id_set)->get_membership(loc),
        highGraph->at(type_id_set)->get_membership(end_loc), type_id_set,
        highGraph->at(type_id_set));
    return u;
}
