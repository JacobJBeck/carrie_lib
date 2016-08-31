// Copyright 2016 Carrie Rebhuhn
#ifndef SINGLEAGENT_IPOLICY_H_
#define SINGLEAGENT_IPOLICY_H_

template<class S, class A, class R>
class IPolicy {
public:
    typedef typename S State;
    typedef typename A Action;
    typedef typename R Reward;

    virtual void update(Reward R) = 0;
    virtual Action operator()(State S) = 0;
};

#endif  // SINGLEAGENT_IPOLICY_H_
