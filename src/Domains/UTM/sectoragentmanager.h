#pragma once

//! Class that manages sectors as agents
class SectorAgentManager : public IAgentManager {
public:
    SectorAgentManager(std::vector<Link*> links_set, size_t n_types_set,
        std::vector<Sector*> sectors_set, size_t set_n_state_elements) :
        IAgentManager(sectors_set.size(), set_n_state_elements), n_types(n_types_set),
        links(links_set), sectors(sectors_set) {
        for (Link* l : links) {
            links_toward_sector[l->target].push_back(l);
        }
    }
    virtual ~SectorAgentManager() {}

    std::vector<Link*> links;  // links in the entire system
    std::map<int, std::vector<Link*> > links_toward_sector;
    size_t n_types;

    virtual matrix2d compute_congestion_state(std::list<UAV*>& UAVs) {
        size_t n_agents = sectors.size();
        matrix2d allStates = easymath::zeros(n_agents, n_state_elements);

        for (UAV* u : UAVs) {
            std::vector<int> sector_congestion_count(n_agents, 0);
            for (UAV* u : UAVs) {
                sector_congestion_count[u->get_cur_sector()]++;
            }
            for (size_t i = 0; i < sectors.size(); i++) {
                for (int conn : sectors[i]->connections) {
                    easymath::XY dx = sectors[i]->xy - sectors[conn]->xy;
                    size_t dir = cardinal_direction(dx);
                    allStates[i][dir]
                        += sector_congestion_count[conn];
                }
            }
        }
        agentStates.push_back(allStates);
        return allStates;
    }

    virtual matrix2d actions2weights(matrix2d agent_actions) {
        // Converts format of agent output to format of A* weights

        matrix2d weights = easymath::zeros(n_types, links.size());
        for (size_t i = 0; i < links.size(); i++) {
            for (size_t j = 0; j < n_types; j++) {
                // type / direction combo
                size_t s = links[i]->source;
                size_t d = j*(n_types - 1) + links[i]->cardinal_dir;

                // turns into type/edge combo
                weights[j][i] = agent_actions[s][d] * 1000.0;
            }
        }
        return weights;
    }
    std::vector<Sector*> sectors;

    void add_delay(UAV* u) {
        // double tt = links.at(u->next_link_ID)->predicted_traversal_time()[0];
        // printf("at link %i: %fs to wait ", u->next_link_ID,tt);
        // system("pause");
        metrics.at(u->get_cur_sector()).local[u->get_type()]++;
    }
    void add_downstream_delay_counterfactual(UAV* u) {
        // remove the effects of the UAV for the counterfactual..
        // calculate the G that means that the UAV's impact is removed...

        if (square_reward) {
            // Nonfunctional, todo.
            printf("SQUARED TODO");
            exit(1);
        } else {
            for (size_t i = 0; i < metrics.size(); i++) {
                if (!u->sector_touched(i)) {
                    metrics[i].G_minus_downstream[u->get_type()]++;
                } else {
                    continue;
                }
            }
        }
    }

    void detect_conflicts() {
        // all links going TO the sector are contribute to its conflict
        for (size_t s = 0; s < sectors.size(); s++) {
            std::vector<Link*> toward = links_toward_sector[s];
            for (size_t i = 0; i < toward.size(); i++) {
                for (size_t j = 0; j < toward[i]->traffic.size(); j++) {
                    int over_capacity = toward[i]->number_over_capacity(j);
                    if (over_capacity <= 0) continue;
                    else if (square_reward)
                        metrics[i].local[j] += over_capacity*over_capacity;
                    else
                        metrics[i].local[j] += over_capacity;
                }
            }
        }
    }
};
