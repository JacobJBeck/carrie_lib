// Copyright 2016 Carrie Rebhuhn
#ifndef SRC_DOMAINS_UTM_SECTORAGENTMANAGER_H_
#define SRC_DOMAINS_UTM_SECTORAGENTMANAGER_H_

#include <map>
#include <list>
#include <vector>

//! Class that manages sectors_ as agents
class SectorAgentManager : public IAgentManager {
 public:
    SectorAgentManager(std::vector<Link*> links, size_t num_types,
        std::vector<Sector*> sectors, size_t num_state_elements) :
        IAgentManager(sectors_.size(), num_state_elements),
        k_num_types_(num_types), links_(links), sectors_(sectors) {
        for (Link* l : links_) {
           k_links_toward_sector_[l->k_target_].push_back(l);
        }
    }
    virtual ~SectorAgentManager() {}

    std::vector<Link*> links_;  // links_ in the entire system
    std::map<int, std::vector<Link*> > k_links_toward_sector_;
    size_t k_num_types_;
    
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

    virtual matrix2d actionsToWeights(matrix2d agent_actions) {
        // Converts format of agent output to format of A* weights

        matrix2d weights = easymath::zeros(k_num_types_, links_.size());
        for (size_t i = 0; i < links_.size(); i++) {
            for (size_t j = 0; j < k_num_types_; j++) {
                // type / direction combo
                size_t s = links_[i]->k_source_;
                size_t d = j*(k_num_types_ - 1) + links_[i]->k_cardinal_dir_;

                // turns into type/edge combo
                weights[j][i] = agent_actions[s][d] * 1000.0;
            }
        }
        return weights;
    }
    std::vector<Sector*> sectors_;

    void addDelay(UAV* u) {
        metrics_.at(u->getNthSector(0)).local_[STATIC_TYPE]++;
    }
    void addDownstreamDelayCounterfactual(UAV* u) {
        // remove the effects of the UAV for the counterfactual..
        // calculate the G that means that the UAV's impact is removed...

        printf("Error -- addDelayDownstreamCounterfactual called. This is disabled. Exiting.");
        system("pause");
        exit(1);
        /*
        if (k_square_reward_mode_) {
            // Nonfunctional, todo.
            printf("SQUARED TODO");
            exit(1);
        } else {
            for (size_t i = 0; i < metrics_.size(); i++) {
                if (!u->sectorTouched(i)) {
                    metrics_[i].g_minus_downstream_[STATIC_TYPE]++;
                } else {
                    continue;
                }
            }
        }*/
    }

    void detectConflicts() {
        // all links_ going TO the sector are contribute to its conflict
        for (size_t s = 0; s < sectors_.size(); s++) {
            std::vector<Link*> toward = k_links_toward_sector_[s];
            for (size_t i = 0; i < toward.size(); i++) {
                for (size_t j = 0; j < toward[i]->traffic_.size(); j++) {
                    int over_capacity = toward[i]->numOverCapacity(j);
                    if (over_capacity <= 0) continue;
                    else if (k_square_reward_mode_)
                        metrics_[i].local_[j] += over_capacity*over_capacity;
                    else
                        metrics_[i].local_[j] += over_capacity;
                }
            }
        }
    }
};
#endif  // SRC_DOMAINS_UTM_SECTORAGENTMANAGER_H_
