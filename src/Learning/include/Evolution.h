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
        m_population = new Population();
        for (Policy* p : E.m_population)
            m_population->push_back(new Policy(*p));
        PopulationMember* m_pop_member_active
            = new PopulationMember(m_population->begin());
    }
    ~Evolution() {
        delete m_pop_member_active;
        easystl::clear(*m_population);
        delete m_population;
    }

    //! Mutators
    virtual void generate_new_members() = 0;
    virtual void activate_next_member() {
        (*m_pop_member_active)++;
        this->set_policy(**m_pop_member_active);
    }
    virtual void select_survivors() = 0;
    void update(const Reward &rwd) { this->policy->update(rwd); }
    void set_first_member() {
        *m_pop_member_active = this->population->begin();
        this->policy = **m_pop_member_active();
    }


    //! Accessor
    bool at_last_member() const {
        return *this->pop_member_active == this->population->end();
    }

 private:
     typedef std::list<Policy*> Population;
     typedef typename std::list<Policy*>::iterator PopulationMember;
     Population* m_population;
     PopulationMember* m_pop_member_active;
};

#endif  // SINGLEAGENT_EVOLUTION_H_
