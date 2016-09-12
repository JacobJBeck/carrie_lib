// Copyright 2016 Carrie Rebhuhn
#ifndef DOMAINS_UTM_UAV_H_
#define DOMAINS_UTM_UAV_H_

// STL includes
#include <string>
#include <list>
#include <set>
#include <utility>

// libraries includes
#include "Planning/include/LinkGraph.h"

class UAV {
    /*
    This class is for moving UAVs in the airspace. They interact with the
    environment through planning. Planning is done through boost.
    */
public:
    typedef size_t UAVType;
    UAV(int start_sector, int end_sector, UAVType t, LinkGraph* highGraph);
    virtual ~UAV() {};
    
    size_t get_next_sector() const { return get_nth_sector(1); }
    virtual size_t get_cur_sector() const { return cur_sector; }
    size_t get_type() const { return type_ID; }
    size_t get_ID() const { return ID; }
    bool at_link_end() const { return t <= 0; }
    bool at_terminal_link() const { return high_path.size() <= 2; }
    size_t get_cur_link() const { return cur_link_ID; }
    void decrement_wait() { t--; }
    size_t get_nth_sector(int n) const { return *std::next(high_path.begin(), n); } // zero indexed
    virtual void planAbstractPath();
    int get_wait() const { return t; }
    void set_wait(int time) { t = time; }
    void set_cur_link_ID(size_t link_ID) { cur_link_ID = link_ID; }
    void set_cur_sector_ID(size_t sID) {
        sectors_touched.insert(sID);
        cur_sector = sID;
    }
    int get_travel_direction() const { return highGraph->get_direction(cur_sector, get_next_sector()); }
    bool link_touched(size_t lID) const { return links_touched.count(lID) > 0; }
    bool sector_touched(size_t sID) const { return sectors_touched.count(sID) > 0; }

protected:
    std::string search_mode;
    std::list<size_t> high_path;
    LinkGraph* highGraph;  //! shared with the simulator (for now)--non-constant subfunctions
    size_t cur_sector;
    const size_t end_sector;
    std::list<size_t> get_best_path() const { return Planning::astar<LinkGraph, size_t>(highGraph, cur_sector, end_sector); }
    size_t next_sector;
    double speed;  // connected to type_ID

    private:
    size_t ID;  //! const in run, but based on non-constant variable

    // Typedefs
    typedef std::pair<size_t, size_t> edge;

    // Constant
    const size_t type_ID;

    // Non-constant
    size_t cur_link_ID;
    int t;
    std::set<size_t> sectors_touched, links_touched;
    

    
    // Accessors
    //! Gets the link ID from the location and desired next loction.
    

    //! Mutators
    //! Sets the current link ID based on passed value
    
    size_t next_sector_ID; // This gets updated after UAV moves
    size_t cur_sector_ID; // for debugging
};
#endif  // DOMAINS_UTM_UAV_H_
