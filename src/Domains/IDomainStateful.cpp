// Copyright 2016 Carrie Rebhuhn
#include "IDomainStateful.h"
#include "yaml-cpp\yaml.h"
#include "UTM/UTMModesAndFiles.h"

IDomainStateful::IDomainStateful(std::string config_file) :
    cur_step(new size_t(0)) {
    YAML::Node configs = YAML::LoadFile(config_file);
    n_steps = configs["constants"]["steps"].as<size_t>();
    n_control_elements = UTMModes::get_n_control_elements(configs);
    n_state_elements = UTMModes::get_n_state_elements(configs);
    n_types = UTMModes::get_n_types(configs);
}


IDomainStateful::~IDomainStateful(void) {
    delete cur_step;
}
