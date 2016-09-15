// Copyright 2016 Carrie Rebhuhn
#ifndef DOMAINS_UTM_UTMMODESANDFILES_H_
#define DOMAINS_UTM_UTMMODESANDFILES_H_

#include <string>
#include "yaml-cpp/yaml.h"
#include <FileIO/include/FileOut.h>

typedef size_t UAVType;

class UTMModes {
 public:
    static size_t getNumTypes(const YAML::Node& config) {
        if (!config["modes"]["types"].as<bool>()) return 1;
        return std::distance(config["types"].begin(), config["types"].end());
    }

    static size_t getNumAgents(const YAML::Node& config, size_t n_links) {
        std::string agent_mode = config["modes"]["agent"].as<std::string>();
        try {
            if (agent_mode == "sector") {
                return config["constants"]["sectors"].as<size_t>();
            } else if (agent_mode == "link") {
                return n_links;
            } else {
                throw std::runtime_error("Bad agent_mode");
            }
        }
        catch (std::runtime_error) {
            printf("Bad agent_mode!");
            exit(1);
        }
    }

    static size_t getNumStateElements(const YAML::Node& config) {
        std::string state_mode = config["modes"]["state"].as<std::string>();
        try {
            if (state_mode == "cardinal")
                return 4;
            else if (state_mode == "single")
                return 1;
            else
                throw std::runtime_error("Bad state_mode");
        }
        catch (std::runtime_error) {
            printf("Bad state_mode!");
            exit(1);
        }
    }
    static size_t getNumControlElements(const YAML::Node& config) {
        return getNumStateElements(config)*getNumTypes(config);
    }
};

class UTMFileNames {
 public:
    static std::string createDomainDirectory(YAML::Node config) {
        std::string n_sectors
            = config["constants"]["sectors"].as<std::string>();
        std::string dir_path = "Domains/" + n_sectors + "_Sectors/"
            + domainNum(config);

        FileOut::mkdir_p(dir_path);
        return dir_path;
    }

    static std::string createExperimentDirectory(std::string config_file) {
        // Creates a directory for the experiment and returns that as a string
        YAML::Node config = YAML::LoadFile("config.yaml");
        std::string agent_defn = config["modes"]["agent"].as<std::string>();
        std::string n_sectors
            = config["constants"]["sectors"].as<std::string>();
        std::string gen_rate
            = config["constants"]["generation_rate"].as<std::string>();
        std::string n_steps = config["constants"]["steps"].as<std::string>();
        std::string n_types = std::to_string(UTMModes::getNumTypes(config));
        std::string reward_mode = config["modes"]["reward"].as<std::string>();
        std::string alpha = config["constants"]["alpha"].as<std::string>();

        std::string dir_path = "Experiments/"
            + agent_defn + "_Agents/"
            + n_sectors + "_Sectors/"
            + "Rate_" + gen_rate + "/"
            + n_steps + "_Steps/"
            + n_types + "_Types/"
            + reward_mode + "_Reward/"
            + alpha + "_alpha/"
            + domainNum(config);

        // Create new directory
        FileOut::mkdir_p(dir_path);

        return dir_path;
    }

 private:
    static std::string domainNum(const YAML::Node& config) {
        if (!config["modes"]["numbered_domain"].as<bool>())
            return "";
        std::string domain_num =
            config["constants"]["domain"].as<std::string>();
        return domain_num + "/";
    }
};
#endif  // DOMAINS_UTM_UTMMODESANDFILES_H_
