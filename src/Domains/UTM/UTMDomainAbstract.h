// Copyright 2016 Carrie Rebhuhn
#ifndef SRC_DOMAINS_UTM_UTMDOMAINABSTRACT_H_
#define SRC_DOMAINS_UTM_UTMDOMAINABSTRACT_H_

#include <utility>
#include <map>
#include <string>
#include <list>
#include <vector>

#include "Domains/IDomainStateful.h"
#include "IAgentManager.h"
#include "Planning/include/MultiGraph.h"
#include "Link.h"
#include "Sector.h"

class UTMDomainAbstract : public IDomainStateful {
 public:
    explicit UTMDomainAbstract(std::string config_file);
    UTMDomainAbstract(std::string config_file, bool only_abstract);
    ~UTMDomainAbstract(void);

 protected:
    typedef std::pair<size_t, size_t> edge;
    IAgentManager* agents_;
    size_t k_num_sectors_;
    MultiGraph<LinkGraph> *high_graph_;
    std::map<edge, size_t> *k_link_ids_;
    std::vector<Link*> links_;
    std::string k_reward_mode_;
    matrix1d num_uavs_at_sector_;
    std::string k_objective_mode_, k_agent_mode_, k_disposal_mode_;
    std::vector<Sector*> sectors_;
    matrix2d link_uavs_;    // The number of UAVs on each link, [step][linkID]
    matrix2d sector_uavs_;  // The number of UAVs waiting at a sector,
                           // [step][sectorID]
    std::list<UAV*> uavs_;
    std::map<int, std::list<UAV*> > uavs_done_;
    std::map<int, std::list<int> > k_incoming_links_;

    void simulateStep(matrix2d agent_actions);
    static bool uavReadyToMove(std::vector<Link*> L,
        std::map<edge, size_t> *L_IDs, UAV *u);
    void generateNewAirspace(std::string dir, size_t xdim, size_t ydim);
    void addLink(edge e, double flat_capacity);
    std::string createExperimentDirectory(std::string config_file);
    virtual void getNewUavTraffic();
    void getNewUavTraffic(int s);
    virtual void absorbUavTraffic();
    matrix2d getStates();
    matrix3d getTypeStates();
    void logStep();
    void exportStepsOfTeam(int team, std::string suffix);
    void exportSectorLocations(int fileID);
    virtual matrix1d getPerformance();
    virtual matrix1d getRewards();
    virtual void incrementUavPath();
    virtual void detectConflicts();
    virtual void getPathPlans();
    virtual void getPathPlans(const std::list<UAV*> &new_uavs);
    virtual void reset();
    virtual void tryToMove(std::vector<UAV*> * eligible_to_move);
    size_t getNthLink(UAV* u, size_t n);
};
#endif  // SRC_DOMAINS_UTM_UTMDOMAINABSTRACT_H_
