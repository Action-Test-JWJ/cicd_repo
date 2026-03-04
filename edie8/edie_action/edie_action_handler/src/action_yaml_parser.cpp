#include "edie_action_handler/action_yaml_parser.hpp"

ActionYamlParser::ActionYamlParser(std::string file_name)
{
    yaml_file_path_ = GetYamlPath(file_name);
    if (LoadYamlData(yaml_file_path_))
    {
        std::cerr << "Predefined Action Loaded Successfully." << std::endl;
    }
    else
    {
        std::cerr << "Failed to Load yaml file Path." << std::endl;
    }
}

ActionYamlParser::~ActionYamlParser() {}

std::string ActionYamlParser::GetYamlPath(std::string file_name)
{
    std::string absolute_path = "";
    absolute_path = ament_index_cpp::get_package_share_directory("edie_action_handler") + "/data/" + file_name;

    return absolute_path;
}

bool ActionYamlParser::LoadYamlData(std::string& file_path)
{
    try 
    {
        if (!fs::exists(file_path)) {
            return false;
        }

        config_ = YAML::LoadFile(file_path);
        return true;
    } 
    catch (const std::exception& e)
    {
        std::cerr << "Exception while loading YAML: " << e.what() << std::endl;
        return false;
    }
}

std::map<int, ActionData> ActionYamlParser::GetActionData()
{
    std::map<int, ActionData> action_map;

    for (const auto& it : config_)
    {
        int action_id = it.first.as<int>();
        YAML::Node node = it.second;

        ActionData data;
        data.display_index = node["display_index"].as<int>();
        data.sound_index   = node["sound_index"].as<int>();
        data.motion_index  = node["motion_index"].as<int>();
        
        // 옵션 배열이 있으면 읽기(없어도 무시)
        if (node["display_options"]) {
            for (const auto& v : node["display_options"]) {
                data.display_options.push_back(v.as<int>());
            }
        }
        if (node["sound_options"]) {
            for (const auto& v : node["sound_options"]) {
                data.sound_options.push_back(v.as<int>());
            }
        }
         
        action_map[action_id] = data;
    }


    return action_map;
}