// Copyright 2016 Carrie Rebhuhn
#include "IDomainStateful.h"
#include <string>

#include "yaml-cpp/yaml.h"


IDomainStateful::IDomainStateful() :
    cur_step_(new size_t(0)) {}


IDomainStateful::~IDomainStateful(void) {
    delete cur_step_;
}
