// Copyright 2016 Carrie Rebhuhn
#ifndef SINGLEAGENT_IPOLICY_H_
#define SINGLEAGENT_IPOLICY_H_

template<class S, class A, class R>
class IPolicy {
public:
    typedef S State;
    typedef A Action;
    typedef R Reward;

    virtual void update(Reward rwd) = 0;
    virtual Action operator()(State stt) = 0;
};

#endif  // SINGLEAGENT_IPOLICY_H_
