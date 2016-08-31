// Copyright 2016 Carrie Rebhuhn

#include "UAV.h"
#include "yaml-cpp/yaml.h"

#include <list>
#include <map>

using std::list;

UAV::UAV(int start_sector, int end_sector_set, UAVType my_type,
    LinkGraph* highGraph) :
    highGraph(highGraph), cur_sector(start_sector), end_sector(end_sector_set),
    type_ID(size_t(my_type)){

    YAML::Node config = YAML::LoadFile("config.yaml");

    speed = std::next(config["types"].begin(), type_ID)->second.as<double>();
    search_mode = config["modes"]["search"].as<std::string>();

    static int calls = 0;
    ID = calls++;

    // Get initial plan and update
    planAbstractPath();
}

void UAV::planAbstractPath() {
    sectors_touched.insert(cur_sector);
    if (search_mode == "astar") {
        high_path = Planning::astar(highGraph, cur_sector, end_sector);
    } else {
        //high_path = highGraph->at(type_ID)->rags(cur_s, end_s);
    }

    if (high_path.empty()) {
        printf("Path not found!");
        system("pause");
    } 
    
    // Set variables
    next_sector = get_next_sector();
}