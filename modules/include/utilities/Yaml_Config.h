#ifndef UTILITIES_YAML_CONFIG_H
#define UTILITIES_YAML_CONFIG_H

#include <yaml-cpp/yaml.h>
#include <fstream>

namespace Yaml_Config {

    extern YAML::Node yaml_param;

    void load_yaml_file_into_param(const char* fileName);  
};
#endif
