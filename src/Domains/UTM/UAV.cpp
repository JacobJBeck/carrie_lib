// Copyright 2016 Carrie Rebhuhn

#include <string>
#include <list>
#include <map>

#include "UAV.h"
#include "yaml-cpp/yaml.h"


using std::list;

UAV::UAV(int start_sector, int end_sector, UAVType my_type,
    LinkGraph* high_graph) :
    high_graph_(high_graph), cur_sector_(start_sector), end_sector_(end_sector),
    k_type_id_(size_t(my_type)) {
    YAML::Node config = YAML::LoadFile("config.yaml");
    if (config["modes"]["types"].as<bool>()) {
        k_speed_
            = std::next(config["types"].begin(), k_type_id_)->second.as<double>();
    } else {
        k_speed_ = 1.0;
    }
    k_search_mode_ = config["modes"]["search"].as<std::string>();

    static int calls = 0;
    k_id_ = calls++;

    // Get initial plan and update
    planAbstractPath();
}

void UAV::reset(int start_sector, int end_sector) {
    cur_sector_ = start_sector;
    end_sector_ = end_sector;

    // Get initial plan and update
    planAbstractPath();
}

void UAV::planAbstractPath() {
    //sectors_touched_.insert(cur_sector_);
    if (k_search_mode_ == "astar") {
        high_path_ = Planning::astar(high_graph_, cur_sector_, end_sector_);
    } else {
        // high_path = highGraph->at(type_ID)->rags(cur_s, end_s);
    }

    if (high_path_.empty()) {
        printf("Path not found!");
        system("pause");
    }

    // Set variables
    next_sector_ = getNthSector(1);
}
