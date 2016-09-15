// Copyright 2016 Carrie Rebhuhn
#ifndef DOMAINS_UTM_SECTOR_H_
#define DOMAINS_UTM_SECTOR_H_

#include "UTMModesAndFiles.h"
#include "Link.h"
#include <vector>
#include <map>
#include "Fix.h"

class Sector {
 public:
    typedef std::pair<size_t,size_t> edge;
    // An area of airspace to control
    Sector(easymath::XY loc, size_t sector_id, std::vector<size_t> connections,
        std::vector<easymath::XY> dest_locs, size_t num_types):
        k_loc_(loc), k_id_(sector_id), k_connections_(connections), k_num_types_(num_types){}
    ~Sector() {}

    // Location properties
    const size_t k_id_;  // the identifier for this sector
    size_t k_num_types_;
    const std::vector<size_t> k_connections_;
    const easymath::XY k_loc_;  // sector center
    Fix* generation_pt_;
};

#endif  // DOMAINS_UTM_SECTOR_H_
