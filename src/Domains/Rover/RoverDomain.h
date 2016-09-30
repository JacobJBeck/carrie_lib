#pragma once
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include "Domains/IDomainStateful.h"

#define MAXPOI 100.0
#define XDIM 100.0
#define YDIM 100.0
#define NPOIS 10
#define NROVS 10
#define RAND double(rand())/double(RAND_MAX)

class POI {
public:
    POI() : value_(RAND*MAXPOI), x_(RAND*XDIM), y_(RAND*YDIM) {}
    double value_, x_, y_;
};

typedef std::vector<double*> Locs;

class Sensor {
public:
    Sensor(double x, double y, int q) : dmin_(1.0), x_(x), y_(y), quadrant_(q) {}
    int quadrant_;

    Locs Lx_, Ly_;


    template< class T>
    static Locs getX(std::vector<T> &obj_arr) {
        Locs xvals;
        for (T &o : obj_arr)
            xvals.push_back(&o.x_);
        return xvals;
    }
    template<class T>
    static Locs getY(std::vector<T> &obj_arr) {
        Locs yvals;
        for (T &o : obj_arr)
            yvals.push_back(&o.y_);
        return yvals;
    }

    void initialize(Locs x, Locs y) {
        Lx_ = x;
        Ly_ = y;
    }

    double denom() {
        double sum = 0;
        for (int i = 0; i < Lx_.size(); i++) {
            if (!inQuadrant(*Lx_[i], *Ly_[i])) continue;
            sum += delta(x_, y_, *Lx_[i], *Ly_[i]);
        }
        return sum;
    }

    virtual double numer() = 0;
    double sense() {
        return numer() / denom();
    }

    double dmin_, x_, y_;
    double delta(double x1, double y1, double x2, double y2) {
        // Squared euclidean norm, bounded by a minimum observation value
        double dx = x1 - x2;
        double dy = y1 - y2;
        return std::min(dx*dx + dy*dy, dmin_*dmin_);
    }
    bool inQuadrant(double x, double y) {
        int val = 2;
        if (x - x_ > 0) {
            if (y - y_ > 0) val = 0;
            else val = 3;
        }
        if (y - y_ > 0) val = 1;
        return val == quadrant_;
    }
};

class POISensor : public Sensor {
public:
    POISensor(double x, double y, int q) : Sensor(x, y, q) {};
    std::vector<double> value_;
    double numer() {
        double sum = 0.0;
        for (int i = 0; i < Lx_.size(); i++) {
            if (!inQuadrant(*Lx_[i], *(Ly_[i]))) continue;
            sum += value_[i];
        }
        return sum;
    }
};

class RoverSensor : public Sensor {
public:
    RoverSensor(double x, double y, int q) : Sensor(x, y, q) {};
    double numer() {
        int sum = 0;
        for (int i = 0; i < Lx_.size(); i++) {
            if (!inQuadrant(*Lx_[i], *Ly_[i])) continue;
            sum++;
        }
        return double(sum);
    }
};

class Rover {
public:
    Rover() : x_(RAND*XDIM), y_(RAND*YDIM) {
        for (int q = 0; q < 4; q++) {
            psensor.push_back(POISensor(x_, y_, q));
            rsensor.push_back(RoverSensor(x_, y_, q));
        }
    };
    std::vector<POISensor> psensor;
    std::vector<RoverSensor> rsensor;
    double x_, y_;
};


class RoverDomain: public IDomainStateful {
public:
    POI __pois[NPOIS];
    Rover __rovers[NROVS];
    std::vector<POI> pois;
    std::vector<Rover> rovers;
    RoverDomain() : IDomainStateful(100, 2, 8, NROVS) {
        pois.assign(__pois, __pois + NPOIS);
        rovers.assign(__rovers, __rovers + NROVS);
        initialize();
    }
    matrix2d getStates() {
        matrix2d S;
        for (auto r : rovers) {
            S.push_back(matrix1d());
            for (auto s : r.psensor) {
                S.back().push_back(s.sense());
            }
            for (auto s : r.rsensor) {
                S.back().push_back(s.sense());
            }
        }
        return S;
    }
    matrix1d getRewards() {
        return matrix1d(k_num_agents_, G());
    }
    matrix1d getPerformance() {
        return getRewards();
    }

    void logStep() {}

    void initialize() {
        Locs px, py, rx, ry;
        px = Sensor::getX(pois);
        py = Sensor::getY(pois);
        rx = Sensor::getX(rovers);
        ry = Sensor::getY(rovers);
        std::vector<double> v;

        for (auto p : pois) {
            v.push_back(p.value_);
        }
        for (auto &r : rovers) {
            for (auto &s : r.psensor) {
                s.initialize(px, py);
                s.value_ = v;
            }
            for (auto &s : r.rsensor) {
                s.initialize(rx, ry);
            }
        }
    }
    double mindist(POI p) {
        double minval = rovers[0].psensor[0].dmin_;
        for (auto r : rovers) {
            minval = std::min(r.psensor[0].delta(r.x_, r.y_, p.x_, p.y_), minval);
        }
        return minval;
    }

    double G() {
        double Gt = 0.0; // g at a particular timestep
        for (auto p : pois) {
            Gt += p.value_ / mindist(p);
        }
        return Gt;
    }

    void simulateStep(matrix2d A) {
        for (int i = 0; i < A.size(); i++) {
            rovers[i].x_ += A[i][0];
            rovers[i].y_ += A[i][1];
        }
    }
    std::string createExperimentDirectory(std::string s) { return ""; }
    void reset() {
        pois.assign(__pois, __pois + NPOIS);
        rovers.assign(__rovers, __rovers + NROVS);
        initialize();
    }
    ~RoverDomain() {};
};



