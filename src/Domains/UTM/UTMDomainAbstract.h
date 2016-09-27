// Copyright 2016 Carrie Rebhuhn
#ifndef SRC_DOMAINS_UTM_UTMDOMAINABSTRACT_H_
#define SRC_DOMAINS_UTM_UTMDOMAINABSTRACT_H_

#include <utility>
#include <map>
#include <string>
#include <list>
#include <vector>

#include "Domains/IDomainStateful.h"
#include "IAgentBody.h"
#include "Planning/include/LinkGraph.h"
#include "Link.h"
#include "Sector.h"

class UTMFileNames {
public:
    static std::string createDomainDirectory(YAML::Node config) {
        std::string n_sectors
            = config["constants"]["sectors"].as<std::string>();
        std::string dir_path = "Domains/" + n_sectors + "_Sectors/"
            + domainNum(config);

        FileOut::mkdir_p(dir_path);
        return dir_path;
    }

    static std::string createExperimentDirectory(std::string config_file) {
        // Creates a directory for the experiment and returns that as a string
        YAML::Node config = YAML::LoadFile("config.yaml");
        std::string agent_defn = config["modes"]["agent"].as<std::string>();
        std::string n_sectors
            = config["constants"]["sectors"].as<std::string>();
        std::string gen_rate
            = config["constants"]["generation_rate"].as<std::string>();
        std::string n_steps = config["constants"]["steps"].as<std::string>();
        std::string reward_mode = config["modes"]["reward"].as<std::string>();
        std::string alpha = config["constants"]["alpha"].as<std::string>();

        std::string dir_path = "Experiments/"
            + agent_defn + "_Agents/"
            + n_sectors + "_Sectors/"
            + "Rate_" + gen_rate + "/"
            + n_steps + "_Steps/"
            + reward_mode + "_Reward/"
            + alpha + "_alpha/"
            + domainNum(config);

        // Create new directory
        FileOut::mkdir_p(dir_path);

        return dir_path;
    }

private:
    static std::string domainNum(const YAML::Node& config) {
        if (!config["modes"]["numbered_domain"].as<bool>())
            return "";
        std::string domain_num =
            config["constants"]["domain"].as<std::string>();
        return domain_num + "/";
    }
};

class UTMDomainAbstract : public IDomainStateful {
 public:
    explicit UTMDomainAbstract(std::string config_file);
    UTMDomainAbstract(std::string config_file, bool only_abstract);
    ~UTMDomainAbstract(void);

 protected:
    typedef std::pair<size_t, size_t> edge;
    IAgentBody* agents_;
    size_t k_num_sectors_;
    LinkGraph *high_graph_;
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
    void logStep();
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
