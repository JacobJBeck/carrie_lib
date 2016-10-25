// Copyright 2016 Carrie Rebhuhn
#ifndef SINGLEAGENT_IPOLICY_H_
#define SINGLEAGENT_IPOLICY_H_

template<class S, class A, class R>
/*
* This is the template class for a policy (state/action/reward mapping). This
* class is pure abstract, and will require overloading for use. The policy is a
* function of a state, and therefore may be accessed as Action = Policy(State).
* An update overload is also provided; the policy is expected to be adaptive.
*/
class IPolicy {
public:
    typedef S State;
    typedef A Action;
    typedef R Reward;

    virtual void update(Reward rwd) = 0;
    virtual Action operator()(State stt) = 0;
};

#endif  // SINGLEAGENT_IPOLICY_H_
