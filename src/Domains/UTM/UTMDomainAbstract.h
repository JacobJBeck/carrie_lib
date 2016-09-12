// Copyright 2016 Carrie Rebhuhn
#ifndef DOMAINS_UTM_UTMDOMAINABSTRACT_H_
#define DOMAINS_UTM_UTMDOMAINABSTRACT_H_

#include <utility>
#include <map>
#include <string>
#include <list>
#include <vector>

#include "Domains/IDomainStateful.h"
#include "Planning/include/RAGS.h"
#include "Planning/include/MultiGraph.h"
#include "IAgentManager.h"
#include "Link.h"
#include "Sector.h"

class UTMDomainAbstract : public IDomainStateful {
 public:
    explicit UTMDomainAbstract(std::string config_file);
    UTMDomainAbstract(std::string config_file, bool only_abstract);
    ~UTMDomainAbstract(void);

 protected:
    IAgentManager* agents;
    size_t n_sectors;
    typedef std::pair<size_t, size_t> edge;
    void simulateStep(matrix2d agent_actions);
    MultiGraph<LinkGraph> *highGraph;
    std::map<edge, size_t> *linkIDs;
    std::vector<Link*> links;
    std::string reward_mode;
    // records number of UAVs at each sector at current time step
    matrix1d numUAVsAtSector;

 private:
    static void get_airspace(YAML::Node configs);
    void generate_new_airspace(std::string dir, size_t n_sectors, size_t xdim, size_t ydim);
    void add_link(edge e, double flat_capacity);

    std::string createExperimentDirectory(std::string config_file) {
        return UTMFileNames::createExperimentDirectory(config_file);
    }

    // Modes
    std::string objective_mode;
    std::string agent_mode;

    // Agents

    // Moving parts
    std::vector<Sector*> sectors;
    std::vector<Fix*> fixes;


    // Traffic
    std::list<UAV*> UAVs;
    virtual void getNewUAVTraffic();
    virtual void absorbUAVTraffic();


    // Graphs/search objects
    RAGS* rags_map;

    // Base function overloads
    matrix2d get_states();
    matrix3d getTypeStates();
    void logStep();
    // The number of UAVs on each link, [step][linkID]

    matrix2d linkUAVs;
    // The number of UAVs waiting at a sector, [step][sectorID]
    matrix2d sectorUAVs;

    void exportStepsOfTeam(int team, std::string suffix);

    void exportSectorLocations(int fileID);
    
    // Different from children
    virtual matrix1d getPerformance();
    virtual matrix1d getRewards();
    virtual void incrementUAVPath();
    virtual void detectConflicts();

    virtual void getPathPlans();
    virtual void getPathPlans(const std::list<UAV*> &new_UAVs);
    virtual void reset();


    //! Moves all it can in the list.s
    // Those eligible to move but who are blocked are left after the function.
    virtual void try_to_move(std::vector<UAV*> * eligible_to_move);


    std::map<int, std::list<UAV*> > UAVs_done;
    std::map<int, std::list<int> > incoming_links;

    size_t get_next_link(UAV* u) {
        return linkIDs->at(edge(u->get_cur_sector(), u->get_next_sector()));
    }
};
#endif  // DOMAINS_UTM_UTMDOMAINABSTRACT_H_
