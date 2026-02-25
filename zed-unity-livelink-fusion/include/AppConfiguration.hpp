#ifndef __APPCONFIGURATION__
#define __APPCONFIGURATION__

#include <string>

struct AppConfiguration
{
    std::string execute_successful_fusion;
    std::string execute_failed_fusion;
    int camera_timeout;

    static AppConfiguration load(const std::string& path);
};

#endif