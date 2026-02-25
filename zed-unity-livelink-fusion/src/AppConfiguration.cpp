#include "AppConfiguration.hpp"
#include "json.hpp"
#include <fstream>

using json = nlohmann::json;

void from_json(const json& j, AppConfiguration& config)
{
    config.execute_successful_fusion = j.value("execute_successful_fusion", "");
    config.execute_failed_fusion = j.value("execute_failed_fusion", "");
    config.camera_timeout = j.value("camera_timeout", 100);
}

AppConfiguration AppConfiguration::load(const std::string& path)
{
    std::ifstream file(path);
    if (!file)
        throw std::runtime_error("File not found: "+path);

    json j;
    file >> j;

    return j.at("AppParameters").get<AppConfiguration>();
}