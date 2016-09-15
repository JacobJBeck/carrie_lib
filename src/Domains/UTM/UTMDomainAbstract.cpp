// Copyright 2016 Carrie Rebhuhn
#include "UTMDomainAbstract.h"

#include <string>
#include <map>
#include <iostream>
#include <list>
#include <vector>

#include "FileIO/include/FileOut.h"
#include "FileIO/include/FileIn.h"
#include "Math/include/easymath.h"
#include "Domains/UTM/sectoragentmanager.h"

using std::string;
using std::list;
using std::vector;
using std::map;
using std::bind;
using std::to_string;
using std::placeholders::_1;
using easyio::file_exists;
using easyio::read_pairs;
using easyio::read2;
using easymath::XY;
using easymath::zeros;

UTMDomainAbstract::UTMDomainAbstract(string config_file, bool) :
    IDomainStateful(config_file) {
    printf("Creating a UTMDomainAbstract object.\n");

    // Load the configurations
    YAML::Node configs = YAML::LoadFile(config_file);

    string domain_dir = UTMFileNames::createDomainDirectory(configs);
    string efile = domain_dir + "edges.csv";
    string vfile = domain_dir + "nodes.csv";
    string airspace_mode = configs["modes"]["airspace"].as<string>();
    k_disposal_mode_ = configs["modes"]["disposal"].as<string>();
    k_num_sectors_ = configs["constants"]["sectors"].as<size_t>();

    // Variables to fill
    if (airspace_mode != "saved" || !file_exists(efile)) {
        size_t xdim = configs["constants"]["xdim"].as<size_t>();
        size_t ydim = configs["constants"]["ydim"].as<size_t>();
        generateNewAirspace(domain_dir, xdim, ydim);
    }

    vector<edge> edges = read_pairs<edge>(efile);
    vector<XY> locs = read_pairs<XY>(vfile);

    LinkGraph* base = new LinkGraph(locs, edges);
    high_graph_ = new MultiGraph<LinkGraph>(k_num_types_, base);

    // Link construction
    double flat_capacity = configs["constants"]["capacity"].as<double>();
    k_link_ids_ = new map<edge, size_t>();
    for (edge e : edges) addLink(e, flat_capacity);

    k_agent_mode_ = configs["modes"]["agent"].as<string>();
    k_num_control_elements_ = UTMModes::getNumControlElements(configs);
    k_num_state_elements_ = UTMModes::getNumStateElements(configs);
    if (k_agent_mode_ == "sector")
        agents_ = new SectorAgentManager(links_, k_num_types_, sectors_,
            k_num_state_elements_);
    else
        agents_ = new LinkAgentManager(links_.size(), k_num_types_, links_,
            k_num_state_elements_);

    k_num_agents_ = UTMModes::getNumAgents(configs, links_.size());
    num_uavs_at_sector_ = zeros(k_num_sectors_);

    k_objective_mode_ = configs["modes"]["objective"].as<string>();
    k_reward_mode_ = configs["modes"]["reward"].as<string>();
}

void UTMDomainAbstract::addLink(UTMDomainAbstract::edge e,
    double flat_capacity) {
    size_t source = e.first;   // membership of origin of edge
    size_t target = e.second;  // membership of connected node
    vector<XY> locs = high_graph_->at()->get_locations();
    XY s_loc = locs[source];
    XY t_loc = locs[target];
    size_t cardinal_dir = cardinal_direction(s_loc - t_loc);
    size_t dist = static_cast<size_t>(euclidean_distance(s_loc, t_loc));
    if (dist == 0)
        dist = 1;
    links_.push_back(
        new Link(links_.size(), source, target, dist,
            vector<size_t>(k_num_types_, static_cast<size_t>(flat_capacity)),
            cardinal_dir, k_num_types_));
    k_link_ids_->insert(make_pair(e, links_.size() - 1));

    k_incoming_links_[target].push_back(source);
}

void UTMDomainAbstract::generateNewAirspace(string domain_dir, size_t xdim,
    size_t ydim) {
    // Generate a new airspace
    LinkGraph* base = new LinkGraph(k_num_sectors_, xdim, ydim);
    MultiGraph<LinkGraph>(k_num_types_, base).at()->print_graph(domain_dir);
}

UTMDomainAbstract::UTMDomainAbstract(string config_file) :
    UTMDomainAbstract(config_file, true) {
    // Sector/Fix  construction
    vector<edge> edges = high_graph_->at()->get_edges();
    vector<vector<size_t> > connections(k_num_sectors_);
    for (edge e : edges)
        connections[e.first].push_back(e.second);

    vector<XY> sector_locs = high_graph_->at()->get_locations();
    for (size_t i = 0; i < k_num_sectors_; i++) {
        Sector* s = new Sector(sector_locs[i], i, connections[i], sector_locs,
            k_num_types_);
        s->generation_pt_ = new Fix(s->k_loc_, s->k_id_, high_graph_,
            sector_locs, k_num_types_);
        sectors_.push_back(s);
    }

    YAML::Node configs = YAML::LoadFile("config.yaml");
    string domain_dir = "Domains/" + to_string(k_num_sectors_) + "_Sectors/";
    if (configs["modes"]["traffic"].as<string>() == "constant") {
        // Create uavs_ on links_
        string pose_file = domain_dir + "initial_pose.csv";
        auto poses = read2<int>(pose_file);
        for (auto p : poses)
            getNewUavTraffic(p[0]);
    }
}

UTMDomainAbstract::~UTMDomainAbstract(void) {
    delete k_link_ids_;
    delete agents_;

    for (Link* l : links_) delete l;
    for (Sector* s : sectors_) delete s;
    for (UAV* u : uavs_) delete u;
    for (size_t s = 0; s < sectors_.size(); s++)
        for (UAV* ud : uavs_done_[s])
            delete ud;
}

matrix1d UTMDomainAbstract::getPerformance() {
    return agents_->performance();
}

matrix1d UTMDomainAbstract::getRewards() {
    return agents_->reward();
}


void UTMDomainAbstract::incrementUavPath() {
    vector<UAV*> eligible;              // uavs_ eligible to move to next link
    copy_if(uavs_.begin(), uavs_.end(), back_inserter(eligible), [](UAV* u) {
        if (u->atLinkEnd()) {
            if (u->atTerminalLink()) {
                return false;
            } else {
                return true;
            }
        } else {
            // TYPE IMPLEMENTATION
            for (size_t i = 0; i <= STATIC_TYPE+1; i++)
                u->decrementWait();
            return false;
        }
    });

    if (eligible.empty()) {
        return;
    } else {
        // This moves all uavs_ that are eligible and not blocked
        tryToMove(&eligible);
        // Only those that cannot move are left in eligible
        // printf("%i uavs_ delayed. \n",eligible.size());

        for (UAV* u : eligible) {
            // adds delay for each eligible UAV not able to move
            agents_->addDelay(u);

            // Add 1 to the sector that the UAV is trying to move from
            num_uavs_at_sector_[u->getNthSector(1)]++;

            // counterfactuals
            if (k_reward_mode_ == "difference_avg")
                agents_->addAverageCounterfactual();
            else if (k_reward_mode_ == "difference_downstream")
                agents_->addDownstreamDelayCounterfactual(u);
            else
                continue;
        }
    }
}

void UTMDomainAbstract::tryToMove(vector<UAV*> * eligible_to_move) {
    random_shuffle(eligible_to_move->begin(), eligible_to_move->end());

    size_t el_size;
    do {
        el_size = eligible_to_move->size();
        easystl::remove_erase_if(eligible_to_move,
            bind(uavReadyToMove, links_, k_link_ids_, _1));
    } while (el_size != eligible_to_move->size());
}

matrix2d UTMDomainAbstract::getStates() {
    /* "NORMAL POLARITY" state
    for (UAV* u : uavs_){
        allStates[getSector(u->loc)][u->get_travel_direction()]+=1.0; // Adds the UAV impact on the state
    }
    */

    // REVERSED POLARITY STATE
    /*for (UAV* u : uavs_){
        // COUNT THE UAV ONLY IF IT HAS A NEXT SECTOR IT'S GOING TO
        if (u->get_next_sector()==u->get_cur_sector()) continue;
        else allStates[u->get_next_sector()][u->get_travel_direction()] += 1.0;
    }*/

    // CONGESTION STATE
    return agents_->computeCongestionState(uavs_);
}


void UTMDomainAbstract::simulateStep(matrix2d agent_actions) {
    // Alter the cost maps (agent actions)
    agents_->logAgentActions(agent_actions);
    bool action_changed = agents_->lastActionDifferent();

    // New uavs_ appear
    if (action_changed) {
        matrix2d w = agents_->actionsToWeights(agent_actions);
        for (size_t i = 0; i < k_num_types_; i++) {
            high_graph_->at(i)->set_weights(w[i]);
        }
    }

    // Note: this adds to traffic
    this->getNewUavTraffic();

    // Make uavs_ reach their destination
    absorbUavTraffic();
    //printf("%i,\n ", uavs_.size());

    // Plan over new cost maps
   // if (action_changed)
        getPathPlans();

    // uavs_ move
    incrementUavPath();
    if (k_objective_mode_ == "conflict")
        detectConflicts();

    //for (auto l: links_) {
    //    std::cout << l->traffic_[0].size() << ",";
        //for (auto t : l->traffic_[0])
         //   std::cout << t->t_ << ",";
    //}
    //std::cout << std::endl;
}

// Records information about a single step in the domain
void UTMDomainAbstract::logStep() {
    if (k_agent_mode_ == "sector" || k_agent_mode_ == "link") {
        matrix1d num_uavs_on_links(links_.size(), 0);
        for (size_t i = 0; i < links_.size(); i++) {
            num_uavs_on_links[i] = links_[i]->traffic_[0].size();
        }
        link_uavs_.push_back(num_uavs_on_links);

        sector_uavs_.push_back(num_uavs_at_sector_);
        num_uavs_at_sector_ = zeros(sectors_.size());
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
void UTMDomainAbstract::exportStepsOfTeam(int team, string suffix) {
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
    std::printf("Error, getTypeStates called. Currently inactive. Exiting.");
    system("pause");
    exit(1);
/*    matrix3d all_states = zeros(k_num_agents_, k_num_types_,
        k_num_state_elements_);

    matrix2d state_printout = zeros(k_num_agents_, k_num_state_elements_);
    if (k_agent_mode_ == "sector") {
        for (UAV* u : uavs_) {
            int a = u->getCurSector();
            size_t id = STATIC_TYPE;
            int dir = u->getTravelDirection();
            all_states[a][id][dir] += 1.0;
            state_printout[a][dir]++;
        }
    } else {
        // link definition
        for (UAV* u : uavs_) {
            size_t a = u->getCurLink();
            size_t id = STATIC_TYPE;
            all_states[a][id][0] += 1.0;
        }
    }
    agents_->agent_states_.push_back(state_printout);
    return all_states;*/
}

bool UTMDomainAbstract::uavReadyToMove(vector<Link*> L,
    map<edge, size_t> *L_IDs, UAV *u) {
    size_t n, c;
    n = L_IDs->at(u->getNthEdge(1)); // Next link ID
    c = L_IDs->at(u->getNthEdge(0)); // Cur link ID

    if (u->atTerminalLink())
        return false;
    if (L[n]->atCapacity(STATIC_TYPE))
        return false;
    u->incrementPath();
    L[n]->moveFrom(u, L[c]);
    return true;
}

void UTMDomainAbstract::exportSectorLocations(int fileID) {
    vector<easymath::XY> sectorLocations;
    for (Sector* s : sectors_)
        sectorLocations.push_back(s->k_loc_);
    FileOut::print_pair_container(sectorLocations,
        "visualization/agent_locations" + to_string(fileID) + ".csv");
}

void UTMDomainAbstract::detectConflicts() {
    agents_->detectConflicts();
    // CURRENTLY CONFLICT DISABLED


    // if (params->_agent_defn_mode==UTMModes::SECTOR){
    // matrix1d G_c(sectors_.size());
    // }

    // count the over capacity here

    // Calculate the amount OVER or UNDER the given capacity
    // COUNT UP SECTOR CAPACITIES

    // matrix2d cap = sectorCapacity;

    // Global reward sectors_
    /*for (UAV* u: uavs_)
        if ((cap[u->get_cur_sector()][u->get_type()]--)<0)
            conflict_count++;

    // D average sectors_
    cap = sectorCapacity;
    for (UAV* u: uavs_)
        if ((cap[u->get_cur_sector()][u->get_type()]--)<0)
            for (int j=0; j<k_num_types_; j++)
                sectors_->at(u->get_cur_sector()).conflicts[j]++; // D avg


    // D downstream sectors_
    cap = sectorCapacity;
    for (UAV* u: uavs_)
        if ((cap[u->get_cur_sector()][u->get_type()]--)<0)
            for (uint i=0; i<sectors_->size(); i++)
                if (u->sectors_touched.find(i)==u->sectors_touched.end())
                    conflict_minus_downstream[i]++;	/// D downstream

    // D reallocation sectors_
    for (uint i=0; i<sectors_->size(); i++){
        matrix2d cap_i = cap;
        matrix1d occ_i = cap[i];
        cap_i[i] = matrix1d(cap_i[0].size(),0);

        for (int j=0; j<k_num_types_; j++){
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

    // Global reward links_
    for (Link* l: links_)
        conflict_count += l->get_conflicts();

    // D average links_
/*	cap = *linkCapacity;
    for (UAV* u: uavs_)
        if ((cap[u->get_cur_link()][u->get_type()]--)<0)
            for (int j=0; j<k_num_types_; j++)
                linkConflicts[u->get_cur_link()][j]++; // D avg
    for (int i=0; i<n_links; i++)
        linkSteps[i]++; // steps of conflict accounting (for average counterfactual)


    // D downstream sectors_
    cap = *linkCapacity;
    for (UAV* u: uavs_)
        if ((cap[u->get_cur_link()][u->type_ID]--)<0)
            for (int i=0; i<n_links; i++)
                if (u->links_touched.find(i)==u->links_touched.end())
                    link_conflict_minus_downstream[i]++;	/// D downstream
                    */
                    // D reallocation sectors_
                    /*
                    for (int i=0; i<n_links; i++){
                        matrix2d cap_i = cap;
                        matrix1d occ_i = cap[i];
                        cap_i[i] = matrix1d(cap_i[0].size(),0);

                        for (int j=0; j<k_num_types_; j++){
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
    for (UAV* u : uavs_) {
        if (!u->atLinkEnd())
            u->planAbstractPath();
    }
}

void UTMDomainAbstract::getPathPlans(const list<UAV*> &new_UAVs) {
    for (UAV* u : new_UAVs) {
        u->planAbstractPath();  // sets own next waypoint
    }
}

void UTMDomainAbstract::reset() {
    sectors_.front()->generation_pt_->reset();
    while (!uavs_.empty()) {
        delete uavs_.back();
        uavs_.pop_back();
    }

    string domain_dir = "Domains/" + to_string(k_num_sectors_) + "_Sectors/";


    for (Link* l : links_) {
        l->reset();
    }
    (*cur_step_) = 0;
    agents_->reset();

    if (k_disposal_mode_ == "keep") {
        string pose_file = domain_dir + "initial_pose.csv";
        auto poses = read2<int>(pose_file);
        for (auto p : poses)
            getNewUavTraffic(p[0]);
    }
}

void UTMDomainAbstract::absorbUavTraffic() {
    for (auto u : uavs_) {
        auto cur_link = k_link_ids_->at(u->getNthEdge(0));
        if (u->atLinkEnd() && u->atTerminalLink()) {
            if (k_disposal_mode_ == "keep") {
                auto cur_sector = u->getNthSector(1);
                u->setCurSector(cur_sector);
                sectors_.at(cur_sector)->generation_pt_->resetUav(u);
                auto new_cur_link = getNthLink(u, 0);
                links_[new_cur_link]->moveFrom(u, links_[cur_link]);
            } else {
                // Remove
                links_.at(cur_link)->remove(u);
                delete u;
                u = NULL;
            }
        }
    }
    easystl::remove_erase_if(&uavs_, [](UAV* u) { return u == NULL; });
}

void UTMDomainAbstract::getNewUavTraffic(int s) {
    UAV* u = sectors_.at(s)->generation_pt_->generateUav();
    auto cur_link = k_link_ids_->at(u->getNthEdge(0));
    links_.at(cur_link)->add(u);
    uavs_.push_back(u);
}

void UTMDomainAbstract::getNewUavTraffic() {
    // Generates (with some probability) plane traffic for each sector
    for (size_t s = 0; s < sectors_.size(); s++) {
        UAV* u = sectors_.at(s)->generation_pt_->generateUav(*cur_step_);
        if (u == NULL) return;

        auto cur_link = k_link_ids_->at(u->getNthEdge(0));
        links_.at(cur_link)->add(u);
        uavs_.push_back(u);
    }
}

size_t UTMDomainAbstract::getNthLink(UAV* u, size_t n) {
    return k_link_ids_->at(u->getNthEdge(n));
}

string UTMDomainAbstract::createExperimentDirectory(string config_file) {
    return UTMFileNames::createExperimentDirectory(config_file);
}
