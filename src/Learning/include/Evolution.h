// Copyright 2016 Carrie Rebhuhn
#ifndef SINGLEAGENT_EVOLUTION_H_
#define SINGLEAGENT_EVOLUTION_H_

#include <list>
#include "IAgent.h"
#include "STL/include/easystl.h"

template <class Policy>
class Evolution : public IAgent<Policy> {
 public:
    //! Life cycle
    Evolution() {}
    Evolution(const Evolution &E) {
        population_ = new Population();
        for (Policy* p : E.population)
            population_->push_back(new Policy(*p));
        PopulationMember* pop_member_active_
            = new PopulationMember(population_->begin());
    }
    ~Evolution() {
        easystl::clear(population_);
    }

    //! Mutators
    virtual void generate_new_members() = 0;
    virtual void activate_next_member() {
        pop_member_active_++;
        this->set_policy(*pop_member_active_);
    }
    virtual void select_survivors() = 0;
    void update(const Reward &rwd) { this->policy->update(rwd); }
    void set_first_member() {
        pop_member_active_ = this->population_->begin();
        this->policy = *pop_member_active();
    }


    //! Accessor
    bool at_last_member() const {
        return *this->pop_member_active_ == this->population_->end();
    }

 protected:
     typedef std::list<Policy*> Population;
     typedef typename std::list<Policy*>::iterator PopulationMember;
     Population population_;
     PopulationMember pop_member_active_;
};

#endif  // SINGLEAGENT_EVOLUTION_H_
