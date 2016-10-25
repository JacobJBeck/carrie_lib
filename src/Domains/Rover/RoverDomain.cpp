// Copyright 2016 Carrie Rebhuhn
#include "Domains\Rover\RoverDomain.h"
#include <algorithm>
#include <vector>

void append(matrix1d &a, const matrix1d &b) {
    a.insert(a.end(), b.begin(), b.end());
}

int Loc::relQuadrant(Loc* l) {
    double dx = l->x_ - x_;
    double dy = l->y_ - y_;

    int val = 2;
    if (l->x_ - x_ > 0) {
        val = 3;
        if (l->y_ - y_ > 0) val = 0;
    }
    if (l->y_ - y_ > 0) val = 1;
    return val;
}

double Loc::norm(Loc *l) {
    double dx = l->x_ - x_;
    double dy = l->y_ - y_;
    return dx*dx + dy*dy;
}

void Loc::operator+=(std::vector<double> v) {
    x_ += v[0];
    y_ += v[1];
}

double Sensor::denom(const std::vector<bool> &in_quadrant) {
    std::vector<double> deltas;
    for (auto l : L_) deltas.push_back(delta(l));
    return ifLeftAddRight(in_quadrant, deltas);
}

double Sensor::sense() {
    std::vector<bool> in_quadrant(inQuadrant(L_));
    double n = numer(in_quadrant);
    if (n == 0) return 0;
    return n / denom(in_quadrant);
}

std::vector<bool> Sensor::inQuadrant(Locs L) {
    std::vector<bool> q;
    for (auto l : L) q.push_back(l_->relQuadrant(l) == quadrant_);
    return q;
}

Rover::Rover() : l_(new Loc(RAND*XDIM, RAND*YDIM)) {
    for (int q = 0; q < 4; q++) {
        psensor.push_back(POISensor(l_, q));
        rsensor.push_back(RoverSensor(l_, q));
    }
}

RoverDomain::RoverDomain() : IDomainStateful(8, 2, NROVS, 100) {
    k_num_agents_ = NROVS;
    state_history_ = matrix2d(k_num_agents_);
    action_history_ = matrix2d(k_num_agents_); // append each time an action taken
    for (int i = 0; i < NPOIS; i++) {
        pois.push_back(new POI());
        poi_start_locs_.push_back(*pois.back()->l_);
    }
    for (int j = 0; j < NROVS; j++) {
        rovers.push_back(new Rover());
        rover_start_locs_.push_back(*rovers.back()->l_);
    }
    initialize();
}

RoverDomain::~RoverDomain() {
    for (auto p : pois) delete p;
    for (auto r : rovers) delete r;
}

matrix2d RoverDomain::getStates() {
    matrix2d S;
    for (auto &r : rovers) {
        S.push_back(matrix1d());
        for (auto s : r->psensor) {
            S.back().push_back(s.sense());
        }
        for (auto s : r->rsensor) {
            S.back().push_back(s.sense());
        }
    }
    return S;
}

void RoverDomain::initialize() {
    Locs P, R;
    P = Sensor::getLocs(pois);
    R = Sensor::getLocs(rovers);
    std::vector<double> v;

    for (auto &p : pois) {
        v.push_back(p->value_);
    }
    for (auto &r : rovers) {
        for (auto &s : r->psensor) {
            s.initialize(P);
            s.value_ = v;
        }
        for (auto &s : r->rsensor) {
            s.initialize(R);
        }
    }
}

double RoverDomain::mindist(POI *p) {
    double minval = rovers[0]->psensor[0].dmin_;
    for (auto &r : rovers) {
        minval = std::max(r->psensor[0].delta(p->l_), minval);
    }
    return minval;
}

double RoverDomain::Gt() {
    double global = 0.0;  // g at a particular timestep
    for (auto &p : pois)
        global += p->value_ / mindist(p);
    return global;
}

void RoverDomain::simulateStep(matrix2d A) {
    matrix2d S = getStates();
    for (uint i = 0; i < A.size(); i++) {
        append(state_history_[i], S[i]);
        append(action_history_[i], A[i]);
        (*rovers[i]->l_) += A[i];
    }
    g_cumulative_ += Gt();
    
    if (*cur_step_ == k_num_steps_ - 1) {
        state_action_log_.push_back(getStateActionHistory());
        G_log.push_back(g_cumulative_);
        resetSAG();
    }
}

void RoverDomain::reset() {
    g_cumulative_ = 0.0;
    for (int i = 0; i < NPOIS; i++) {
        (*pois[i]->l_) = poi_start_locs_[i];
    }
    for (int j = 0; j < NROVS; j++) {
        (*rovers[j]->l_) = rover_start_locs_[j];
    }

    initialize();
    (*cur_step_) = 0;
}
