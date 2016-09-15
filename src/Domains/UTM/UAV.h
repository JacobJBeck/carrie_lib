// Copyright 2016 Carrie Rebhuhn
#ifndef SRC_DOMAINS_UTM_UAV_H_
#define SRC_DOMAINS_UTM_UAV_H_

// STL includes
#include <string>
#include <list>
#include <set>
#include <utility>

// libraries includes
#include "Planning/include/LinkGraph.h"

#define STATIC_TYPE 0 // replaces u->getType()

class UAV {
    /*
    This class is for moving UAVs in the airspace. They interact with the
    environment through planning. Planning is done through boost.
    */
public:
    typedef size_t UAVType;
    UAV(int start_sector, int end_sector, UAVType t, LinkGraph* high_graph);
    virtual ~UAV() {};
    void reset(int start_sector, int end_sector);
    // Gets the zero-indexed nth edge in the path
    size_t getNthSector(int n) const {
        if (high_path_.size() <= n) {
            return high_path_.front();
        }
        return *std::next(high_path_.begin(), n);  // zero indexed
    }
    std::pair<size_t, size_t> getNthEdge(size_t n) {
        return std::make_pair(getNthSector(n), getNthSector(n + 1));
    }
    void incrementPath() {
        high_path_.pop_front();
        cur_sector_ = high_path_.front();
    }

    virtual void planAbstractPath();
    bool atLinkEnd() const { return t_ <= 0; }
    bool atTerminalLink() const { return high_path_.size() <= 2; }
    int getWait() const { return t_; }
    void setWait(int time) { t_ = time; }
    void decrementWait() { t_--; }
    void setCurSector(size_t s) {
        cur_sector_ = s;
    }
    size_t getId() const { return k_id_; }
private:


    //size_t getNextSector() const { return getNthSector(1); }
    //virtual size_t getCurSector() const { return cur_sector_; }
    //size_t getType() const { return k_type_id_; }
    //size_t getCurLink() const { return cur_link_id_; }
    //void setCurLinkId(size_t link_id) { cur_link_id_ = link_id; }
    //void setCurSectorId(size_t sId) {
    //    sectors_touched_.insert(sId);
    //    cur_sector_ = sId;
    //}



    int getTravelDirection() const {
        return high_graph_->get_direction(cur_sector_, getNthSector(1));
    }
    //bool linkTouched(size_t lId) const { return links_touched_.count(lId) > 0; }
    //bool sectorTouched(size_t sId) const {
    //    return sectors_touched_.count(sId) > 0;
    //}

 //protected:
    std::string k_search_mode_;
    std::list<size_t> high_path_;
    LinkGraph* high_graph_;
    size_t cur_sector_;
    size_t end_sector_;
    std::list<size_t> getBestPath() const {
        return Planning::astar<LinkGraph, size_t>
            (high_graph_, cur_sector_, end_sector_);
    }
    size_t next_sector_;
    double k_speed_;  // connected to type_ID

 //private:
    size_t k_id_;  //! const in run, but based on non-constant variable

    // Typedefs
    typedef std::pair<size_t, size_t> edge;

    // Constant
    const size_t k_type_id_;

    // Non-constant
    //size_t cur_link_id_;

    int t_;
    //std::set<size_t> sectors_touched_, links_touched_;

    size_t next_sector_id_;  // This gets updated after UAV moves
    size_t cur_sector_id_;   // for debugging
};
#endif  // SRC_DOMAINS_UTM_UAV_H_
