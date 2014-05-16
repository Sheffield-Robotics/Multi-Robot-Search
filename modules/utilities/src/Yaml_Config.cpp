#include "utilities/Yaml_Config.h"

namespace Yaml_Config
{
    YAML::Node yaml_param;
    
    void load_yaml_file_into_param(const char* fileName) 
    {
        yaml_param = YAML::LoadFile(fileName);
    }
};

 