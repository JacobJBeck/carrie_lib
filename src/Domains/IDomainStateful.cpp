// Copyright 2016 Carrie Rebhuhn
#include "IDomainStateful.h"
#include "yaml-cpp/yaml.h"
#include "UTM/UTMModesAndFiles.h"

IDomainStateful::IDomainStateful(std::string config_file) :
    cur_step_(new size_t(0)) {
    YAML::Node configs = YAML::LoadFile(config_file);
    k_num_steps_ = configs["constants"]["steps"].as<size_t>();
    k_num_control_elements_ = UTMModes::getNumControlElements(configs);
    k_num_state_elements_ = UTMModes::getNumStateElements(configs);
    k_num_types_ = UTMModes::getNumTypes(configs);
}


IDomainStateful::~IDomainStateful(void) {
    delete cur_step_;
}
