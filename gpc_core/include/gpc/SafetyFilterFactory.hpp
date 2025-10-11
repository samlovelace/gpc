#ifndef SAFETYFILTERFACTORY_HPP
#define SAFETYFILTERFACTORY_HPP
 
#include <string> 
#include <memory>
#include "gpc/ConfigManager.hpp"
#include "gpc/ISafetyFilter.hpp"

#include "gpc/QpSafetyFilter.h"
 
class SafetyFilterFactory 
{ 
public:
    SafetyFilterFactory();
    ~SafetyFilterFactory();

    static inline std::shared_ptr<ISafetyFilter> create(const std::string& aFullConfigFilePath)
    {
        YAML::Node safetyFilterConfig; 
        if(!ConfigManager::get().getConfig<YAML::Node>("SafetyFilter", safetyFilterConfig, ConfigManager::get().getFullConfig()))
        {
            std::cerr << "Invalid or missing SafetyFilter configuration" << std::endl; 
            return nullptr; 
        }

        std::string safetyFilterType; 
        if(!ConfigManager::get().getConfig<std::string>("type", safetyFilterType, safetyFilterConfig))
        {
            std::cerr << "Invalid or missing SafetyFilter type" << std::endl; 
            return nullptr; 
        }

        std::string safetyFilterConfigFile; 
        if(!ConfigManager::get().getConfig<std::string>("file", safetyFilterConfigFile, safetyFilterConfig))
        {
            std::cerr << "Invalid or missing SafetyFilter config file" << std::endl; 
            return nullptr; 
        }

        std::string fullpath = aFullConfigFilePath + safetyFilterConfigFile; 
        YAML::Node safetyFilterSpecificConfig; 
        if(!ConfigManager::get().load(fullpath, safetyFilterSpecificConfig))
        {
            std::cerr << "Invalid or missing SafetyFilter specific config file" << std::endl; 
            return nullptr; 
        }

        return create(safetyFilterType, safetyFilterSpecificConfig); 
    }

    static inline std::shared_ptr<ISafetyFilter> create(const std::string& aSafetyFilterType, const YAML::Node& aSafetyFilterSpecificConfig)
    {
        if("qp" == aSafetyFilterType || "QP" == aSafetyFilterType)
        {
            return std::make_shared<QpSafetyFilter>();  
        }
        else
        {
            std::cout << "Unsupported SafetyFilter type: " << aSafetyFilterType << std::endl; 
            return nullptr; 
        }
    }


private:
   
};
#endif //SAFETYFILTERFACTORY_HPP