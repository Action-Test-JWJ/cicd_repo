#ifndef ACTION_YAML_PARSER_HPP
#define ACTION_YAML_PARSER_HPP

#include "yaml-cpp/yaml.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <iostream>
#include <filesystem>
#include "action_data.hpp" 

namespace fs = std::filesystem;

class ActionYamlParser
{
public:
    ActionYamlParser(std::string file_name);
    ~ActionYamlParser();
    
    std::map<int, ActionData> GetActionData();

private:
    bool LoadYamlData(std::string& file_path);
    std::string GetYamlPath(std::string file_name);
    void PrintData(uint8_t index);
    std::string yaml_file_path_;
    YAML::Node config_;
};

#endif // ACTION_YAML_PARSER_HPP