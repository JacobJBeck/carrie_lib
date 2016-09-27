// Copyright 2016 Carrie Rebhuhn

#include <string>
#include <list>
#include <map>

#include "UAV.h"
#include "yaml-cpp/yaml.h"


using std::list;

UAV::UAV(int start_sector, int end_sector, LinkGraph* high_graph) :
    high_graph_(high_graph), cur_sector_(start_sector), end_sector_(end_sector) {
    YAML::Node config = YAML::LoadFile("config.yaml");
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
    if (k_search_mode_ == "astar")
        high_path_ = Planning::astar(high_graph_, cur_sector_, end_sector_);


    if (high_path_.empty()) {
        printf("Path not found!");
        system("pause");
    }

    // Set variables
    next_sector_ = getNthSector(1);
}
