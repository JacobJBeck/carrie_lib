// Copyright 2016 Carrie Rebhuhn
#ifndef SRC_DOMAINS_UTM_SECTORAGENTMANAGER_H_
#define SRC_DOMAINS_UTM_SECTORAGENTMANAGER_H_

#include <map>
#include <list>
#include <vector>

//! Class that manages sectors_ as agents
class SectorAgentManager : public IAgentManager {
 public:
    SectorAgentManager(std::vector<Link*> links,
        std::vector<Sector*> sectors, size_t num_state_elements) :
        IAgentManager(sectors_.size(), num_state_elements),
        links_(links), sectors_(sectors) {
        for (Link* l : links_) {
           k_links_toward_sector_[l->k_target_].push_back(l);
        }
    }
    virtual ~SectorAgentManager() {}

    std::vector<Link*> links_;  // links_ in the entire system
    std::map<int, std::vector<Link*> > k_links_toward_sector_;
    
    matrix2d computeCongestionState(const std::list<UAV*>& uavs) {
        size_t n_agents = sectors_.size();
        matrix2d allStates = easymath::zeros(n_agents, k_num_state_elements_);

        for (UAV* u : uavs) {
            std::vector<int> sector_congestion_count(n_agents, 0);
            for (UAV* u : uavs) {
                sector_congestion_count[u->getNthSector(0)]++;
            }
            for (size_t i = 0; i < sectors_.size(); i++) {
                for (int conn : sectors_[i]->k_connections_) {
                    easymath::XY dx =
                        sectors_[i]->k_loc_ - sectors_[conn]->k_loc_;
                    size_t dir = cardinal_direction(dx);
                    allStates[i][dir]
                        += sector_congestion_count[conn];
                }
            }
        }
        agent_states_.push_back(allStates);
        return allStates;
    }

    virtual matrix1d actionsToWeights(matrix2d agent_actions) {
        // Converts format of agent output to format of A* weights

        matrix1d weights = easymath::zeros(links_.size());
        for (size_t i = 0; i < links_.size(); i++) {
            size_t s = links_[i]->k_source_;
            size_t d = links_[i]->k_cardinal_dir_;

            weights[i] = agent_actions[s][d] * 1000.0;
        }
        return weights;
    }
    std::vector<Sector*> sectors_;

    void addDelay(UAV* u) {
        metrics_.at(u->getNthSector(0)).local_++;
    }
    void addDownstreamDelayCounterfactual(UAV* u) {
        // remove the effects of the UAV for the counterfactual..
        // calculate the G that means that the UAV's impact is removed...

        printf("Error -- addDelayDownstreamCounterfactual called. This is disabled. Exiting.");
        system("pause");
        exit(1);
    }

    void detectConflicts() {
        // all links_ going TO the sector are contribute to its conflict
        for (size_t s = 0; s < sectors_.size(); s++) {
            std::vector<Link*> toward = k_links_toward_sector_[s];
            for (size_t i = 0; i < toward.size(); i++) {
                int over_capacity = toward[i]->numOverCapacity();
                if (over_capacity <= 0) continue;
                else if (k_square_reward_mode_)
                    metrics_[i].local_ += over_capacity*over_capacity;
                else
                    metrics_[i].local_ += over_capacity;
            }
        }
    }
};
#endif  // SRC_DOMAINS_UTM_SECTORAGENTMANAGER_H_
