// Copyright 2016 Carrie Rebhuhn
#include "UTMDomainAbstract.h"
#include <vector>
#include <string>
#include <list>
#include <map>

#include "UTMModesAndFiles.h"
#include "FileIO/include/FileIn.h"
#include "Domains/UTM/sectoragentmanager.h"

using std::list;
using std::vector;
using std::string;
using easymath::XY;
using std::ifstream;
using std::map;
using easymath::zeros;

UTMDomainAbstract::UTMDomainAbstract(std::string config_file, bool) :
    IDomainStateful(config_file) {
    printf("Creating a UTMDomainAbstract object.\n");

    // Load the configurations
    YAML::Node configs = YAML::LoadFile(config_file);

    string domain_dir = UTMFileNames::createDomainDirectory(configs);
    string efile = domain_dir + "edges.csv";
    string vfile = domain_dir + "nodes.csv";
    string airspace_mode = configs["modes"]["airspace"].as<std::string>();
    n_sectors = configs["constants"]["sectors"].as<size_t>();
    
    // Variables to fill 
    if (airspace_mode != "saved" || !easyio::file_exists(efile)) {
        size_t xdim = configs["constants"]["xdim"].as<size_t>();
        size_t ydim = configs["constants"]["ydim"].as<size_t>();
        generate_new_airspace(domain_dir, n_sectors, xdim, ydim);
    }

    vector<edge> edges = easyio::read_pairs<edge>(efile);
    vector<XY> locs = easyio::read_pairs<XY>(vfile);

    LinkGraph* base = new LinkGraph(locs, edges);
    highGraph = new MultiGraph<LinkGraph>(n_types, base);

    // Link construction
    double flat_capacity = configs["constants"]["capacity"].as<double>();
    linkIDs = new map<edge, size_t>();
    for (edge e : edges) add_link(e, flat_capacity);

    agent_mode = configs["modes"]["agent"].as<std::string>();
    n_control_elements = UTMModes::get_n_control_elements(configs);
    n_state_elements = UTMModes::get_n_state_elements(configs);
    if (agent_mode == "sector")
        agents = new SectorAgentManager(links, n_types, sectors, n_state_elements);
    else
        agents = new LinkAgentManager(links.size(), n_types, links, n_state_elements);

    n_agents = UTMModes::get_n_agents(configs, links.size());
    numUAVsAtSector = zeros(n_sectors);

    objective_mode = configs["modes"]["objective"].as<std::string>();
    reward_mode = configs["modes"]["reward"].as<std::string>();

}

void UTMDomainAbstract::add_link(UTMDomainAbstract::edge e, double flat_capacity) {
    size_t source = e.first;   // membership of origin of edge
    size_t target = e.second;  // membership of connected node
    vector<XY> locs = highGraph->at()->get_locations();
    XY s_loc = locs[source];
    XY t_loc = locs[target];
    size_t cardinal_dir = cardinal_direction(s_loc - t_loc);
    size_t dist = static_cast<size_t>(euclidean_distance(s_loc, t_loc));
    if (dist == 0) dist = 1;
    links.push_back(
        new Link(links.size(), source, target, dist,
            vector<size_t>(n_types, static_cast<size_t>(flat_capacity)), cardinal_dir, n_types));
    linkIDs->insert(std::make_pair(e, links.size() - 1));

    incoming_links[target].push_back(source);
}

void UTMDomainAbstract::generate_new_airspace(string domain_dir, size_t n_sectors, size_t xdim, size_t ydim) {
    // Generate a new airspace
    LinkGraph* base = new LinkGraph(n_sectors, xdim, ydim);
    MultiGraph<LinkGraph>(n_types, base).at()->print_graph(domain_dir);
}

UTMDomainAbstract::UTMDomainAbstract(string config_file) : UTMDomainAbstract(config_file, true) {
    // Sector/Fix  construction
    vector<edge> edges = highGraph->at()->get_edges();
    vector<vector<size_t> > connections(n_sectors);
    for (edge e : edges)
        connections[e.first].push_back(e.second);

    vector<XY> sector_locs = highGraph->at()->get_locations();
    for (size_t i = 0; i < n_sectors; i++) {
        Sector* s = new Sector(sector_locs[i], i, connections[i], sector_locs, n_types);
        s->generation_pt = new Fix(s->xy, s->ID, highGraph, sector_locs, n_types);
        sectors.push_back(s);
    }
    
    YAML::Node configs = YAML::LoadFile("config.yaml");
    if (configs["modes"]["traffic"].as<std::string>() == "constant") {
        size_t n_uavs = configs["constants"]["vehicles"].as<size_t>();
        for (size_t i = 0; i < n_uavs; i++) {
            // Create UAVs on links
            std::vector<std::vector<int> > poses = easyio::read2<int>("Domains/" + std::to_string(n_sectors) + "_Sectors/initial_pose.csv");
            for (std::vector<int> p : poses) {
                UAVs.push_back(sectors.at(p[0])->generation_pt->generate_UAV());
            }
        }
    }
}

UTMDomainAbstract::~UTMDomainAbstract(void) {
    delete linkIDs;
    delete agents;

    for (Link* l : links) delete l;
    for (Sector* s : sectors) delete s;
    for (UAV* u : UAVs) delete u;
    for (size_t s = 0; s < sectors.size(); s++)
        for (UAV* ud : UAVs_done[s])
            delete ud;
}

matrix1d UTMDomainAbstract::getPerformance() {
    return agents->performance();
}

matrix1d UTMDomainAbstract::getRewards() {
    return agents->reward();
}


void UTMDomainAbstract::incrementUAVPath() {
    vector<UAV*> eligible;              // UAVs eligible to move to next link
    copy_if(UAVs.begin(), UAVs.end(), back_inserter(eligible), [](UAV* u) {
        if (u->at_link_end()) {
            if (u->at_terminal_link()) {
                return false;
            } else {
                return true;
            }
        } else {
            // TYPE IMPLEMENTATION
            for (size_t i = 0; i <= u->get_type(); i++)
                u->decrement_wait();
            return false;
        }
    });

    if (eligible.empty()) {
        return;
    } else {
        // This moves all UAVs that are eligible and not blocked
        try_to_move(&eligible);
        // Only those that cannot move are left in eligible
        // std::printf("%i UAVs delayed. \n",eligible.size());

        for (UAV* u : eligible) {
            // adds delay for each eligible UAV not able to move
            agents->add_delay(u);

            // Add 1 to the sector that the UAV is trying to move from
            numUAVsAtSector[u->get_next_sector()]++;

            // counterfactuals
            if (reward_mode == "difference_avg")
                agents->add_average_counterfactual();
            else if (reward_mode == "difference_downstream")
                agents->add_downstream_delay_counterfactual(u);
            else
                continue;
        }
    }
}

void UTMDomainAbstract::try_to_move(vector<UAV*> * eligible_to_move) {
    random_shuffle(eligible_to_move->begin(), eligible_to_move->end());

    size_t el_size;
    do {
        el_size = eligible_to_move->size();

        vector<Link*> L = links;

        map<edge, size_t>* L_IDs = linkIDs;
        eligible_to_move->erase(
            remove_if(eligible_to_move->begin(), eligible_to_move->end(),
                [L, L_IDs](UAV* u) {
            if (u->at_terminal_link()) {
                return false;
            }

            // Get next link ID
            size_t n = L_IDs->at(edge(u->get_nth_sector(1), u->get_nth_sector(2)));
            // Get current link ID
            size_t c = u->get_cur_link();
            size_t t = u->get_type();

            if (!L[n]->at_capacity(t)) {
                L[n]->move_from(u, L[c]);
                return true;
            } else {
                return false;
            } }),
            eligible_to_move->end());
    } while (el_size != eligible_to_move->size());
}

matrix2d UTMDomainAbstract::get_states() {


    /* "NORMAL POLARITY" state
    for (UAV* u : UAVs){
        allStates[getSector(u->loc)][u->get_travel_direction()]+=1.0; // Adds the UAV impact on the state
    }
    */

    // REVERSED POLARITY STATE
    /*for (UAV* u : UAVs){
        // COUNT THE UAV ONLY IF IT HAS A NEXT SECTOR IT'S GOING TO
        if (u->get_next_sector()==u->get_cur_sector()) continue;
        else allStates[u->get_next_sector()][u->get_travel_direction()] += 1.0;
    }*/

    // CONGESTION STATE
    return agents->compute_congestion_state(UAVs);
}


void UTMDomainAbstract::simulateStep(matrix2d agent_actions) {
    // Alter the cost maps (agent actions)
    agents->logAgentActions(agent_actions);
    bool action_changed = agents->last_action_different();

    // New UAVs appear
    if (action_changed) {
        matrix2d w = agents->actions2weights(agent_actions);
        for (size_t i = 0; i < n_types; i++) {
            highGraph->at(i)->set_weights(w[i]);
        }
    }

    // Note: this adds to traffic
    this->getNewUAVTraffic();

    // Make UAVs reach their destination
    absorbUAVTraffic();
    std::printf("%i,\n ", UAVs.size());
    system("pause");

    // Plan over new cost maps
    if (action_changed)
        getPathPlans();

    // UAVs move
    incrementUAVPath();
    if (objective_mode == "conflict")
        detectConflicts();

}

// Records information about a single step in the domain
void UTMDomainAbstract::logStep() {
    if (agent_mode == "sector" || agent_mode == "link") {
        matrix1d numUAVsOnLinks(links.size(), 0);
        for (size_t i = 0; i < links.size(); i++) {
            numUAVsOnLinks[i] = links[i]->traffic[0].size();
        }
        linkUAVs.push_back(numUAVsOnLinks);

        sectorUAVs.push_back(numUAVsAtSector);
        numUAVsAtSector = zeros(sectors.size());
    }
}

// This function is called in SimNE.cpp. The first argument is the number
// of the NN team (out of 20 or w/e pop size is) with the best performance.
// Second arg is used to differentiate between different points in evolution
// (trained or untrained, for example).
// This function is called once per full simulation (words?), after the last
// epoch
// TODO(Brandon) -- we should probably dump this data to a file before
// moving to the next neural evaluation
void UTMDomainAbstract::exportStepsOfTeam(int team, std::string suffix) {
    /* -- not compiling for now
    if (linkUAVs.size() == 0) return;
    size_t start = n_steps*team;
    matrix2d link_log, sector_log;
    for (size_t i = 0; i < n_steps; i++) {
        link_log.push_back(linkUAVs[start + i]);
        sector_log.push_back(sectorUAVs[start + i]);
    }

    // Save history of traffic in csv files
    // Maybe it's good to change where these are saved
    FileOut::print_vector(link_log, "link_UAVs_" + suffix + ".csv");
    FileOut::print_vector(sector_log, "sector_UAVs_" + suffix + ".csv");
    linkUAVs.clear();
    sectorUAVs.clear();
    */
}

matrix3d UTMDomainAbstract::getTypeStates() {
    matrix3d allStates = zeros(n_agents, n_types, n_state_elements);

    matrix2d state_printout = zeros(n_agents, n_state_elements);
    if (agent_mode == "sector") {
        for (UAV* u : UAVs) {
            int a = u->get_cur_sector();
            size_t id = u->get_type();
            int dir = u->get_travel_direction();
            allStates[a][id][dir] += 1.0;
            state_printout[a][dir]++;
        }
    } else {
        // link definition
        for (UAV* u : UAVs) {
            size_t a = u->get_cur_link();
            size_t id = u->get_type();
            allStates[a][id][0] += 1.0;
        }
    }
    agents->agentStates.push_back(state_printout);
    return allStates;
}

void UTMDomainAbstract::exportSectorLocations(int fileID) {
    std::vector<easymath::XY> sectorLocations;
    for (Sector* s : sectors)
        sectorLocations.push_back(s->xy);
    FileOut::print_pair_container(sectorLocations,
        "visualization/agent_locations" + std::to_string(fileID) + ".csv");
}

void UTMDomainAbstract::detectConflicts() {
    agents->detect_conflicts();
    // CURRENTLY CONFLICT DISABLED


    // if (params->_agent_defn_mode==UTMModes::SECTOR){
    // matrix1d G_c(sectors.size());
    // }

    // count the over capacity here

    // Calculate the amount OVER or UNDER the given capacity
    // COUNT UP SECTOR CAPACITIES

    // matrix2d cap = sectorCapacity;

    // Global reward SECTORS
    /*for (UAV* u: UAVs)
        if ((cap[u->get_cur_sector()][u->get_type()]--)<0)
            conflict_count++;

    // D average SECTORS
    cap = sectorCapacity;
    for (UAV* u: UAVs)
        if ((cap[u->get_cur_sector()][u->get_type()]--)<0)
            for (int j=0; j<n_types; j++)
                sectors->at(u->get_cur_sector()).conflicts[j]++; // D avg


    // D downstream SECTORS
    cap = sectorCapacity;
    for (UAV* u: UAVs)
        if ((cap[u->get_cur_sector()][u->get_type()]--)<0)
            for (uint i=0; i<sectors->size(); i++)
                if (u->sectors_touched.find(i)==u->sectors_touched.end())
                    conflict_minus_downstream[i]++;	/// D downstream

    // D reallocation SECTORS
    for (uint i=0; i<sectors->size(); i++){
        matrix2d cap_i = cap;
        matrix1d occ_i = cap[i];
        cap_i[i] = matrix1d(cap_i[0].size(),0);

        for (int j=0; j<n_types; j++){
            while (occ_i[j]<0){ // ONLY TAKE OUT THE OVER CAPACITY--ASSUME PERFECT ROUTING?
                // add back up to capacity
                occ_i[j]++;

                int alt;
                do {
                    alt = rand()%n_agents;
                } while(alt==i);
                cap_i[alt][j]--;
            }
        }
        for (uint j=0; j<cap_i.size(); j++)
            for (uint k=0; k<cap_i[j].size(); k++)
                if (cap_i[j][k]<0)
                    conflict_random_reallocation[i]++;
    }

    // Global reward LINKS
    for (Link* l: links)
        conflict_count += l->get_conflicts();

    // D average LINKS
/*	cap = *linkCapacity;
    for (UAV* u: UAVs)
        if ((cap[u->get_cur_link()][u->get_type()]--)<0)
            for (int j=0; j<n_types; j++)
                linkConflicts[u->get_cur_link()][j]++; // D avg
    for (int i=0; i<n_links; i++)
        linkSteps[i]++; // steps of conflict accounting (for average counterfactual)


    // D downstream SECTORS
    cap = *linkCapacity;
    for (UAV* u: UAVs)
        if ((cap[u->get_cur_link()][u->type_ID]--)<0)
            for (int i=0; i<n_links; i++)
                if (u->links_touched.find(i)==u->links_touched.end())
                    link_conflict_minus_downstream[i]++;	/// D downstream
                    */
                    // D reallocation SECTORS
                    /*
                    for (int i=0; i<n_links; i++){
                        matrix2d cap_i = cap;
                        matrix1d occ_i = cap[i];
                        cap_i[i] = matrix1d(cap_i[0].size(),0);

                        for (int j=0; j<n_types; j++){
                            while (occ_i[j]<0){ // ONLY TAKE OUT THE OVER CAPACITY--ASSUME PERFECT ROUTING?
                                // add back up to capacity
                                occ_i[j]++;

                                int alt;
                                do {
                                    alt = rand()%n_links;
                                } while(alt==i);
                                cap_i[alt][j]--;
                            }
                        }
                        for (uint j=0; j<cap_i.size(); j++)
                            for (uint k=0; k<cap_i[j].size(); k++)
                                if (cap_i[j][k]<0)
                                    link_conflict_(om_reallocation[i]++;
                    }
                    */
}

void UTMDomainAbstract::getPathPlans() {
    for (UAV* u : UAVs) {
        if (u->at_link_end())
            u->planAbstractPath();
    }
}

void UTMDomainAbstract::getPathPlans(const std::list<UAV* > &new_UAVs) {
    for (UAV* u : new_UAVs) {
        u->planAbstractPath();  // sets own next waypoint
    }
}

void UTMDomainAbstract::reset() {
    sectors.front()->generation_pt->reset();
    while (!UAVs.empty()) {
        delete UAVs.back();
        UAVs.pop_back();
    }

    for (Link* l : links) {
        l->reset();
    }

    agents->reset();
}

void UTMDomainAbstract::absorbUAVTraffic() {
    // Deletes UAVs
    vector<Link*> l = links;

    easystl::remove_erase_if(&UAVs, [l](UAV* u) {
        if (u->at_link_end() && u->at_terminal_link()) {
            l[u->get_cur_link()]->remove(u);
            delete u;
            return true;
        } else {
            return false;
        }
    });
}


void UTMDomainAbstract::getNewUAVTraffic() {
    // Generates (with some probability) plane traffic for each sector
    for (Sector* s : sectors) {
        UAV* u = s->generation_pt->generate_UAV(*cur_step);
        if (u == NULL) continue;

        // UAV initially does not have its current link set -- must define from sector pair
        edge u_start_edge = edge(u->get_cur_sector(), u->get_next_sector());
        u->set_cur_link_ID(linkIDs->at(u_start_edge));
        links.at(u->get_cur_link())->add(u);
        UAVs.push_back(u);
    }
}
