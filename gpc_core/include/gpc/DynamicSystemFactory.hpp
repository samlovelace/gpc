#ifndef DYNAMICSYSTEMFACTORY_HPP
#define DYNAMICSYSTEMFACTORY_HPP
 
#include <memory>
#include <yaml-cpp/yaml.h>
#include "IDynamicSystem.hpp" 
#include "ConfigManager.hpp"

#include "LinearSystem.h"

class DynamicSystemFactory 
{ 
public:
    DynamicSystemFactory();
    ~DynamicSystemFactory();

    static inline std::shared_ptr<IDynamicSystem> create(const std::string& aFullConfigFilePath)
    {
        YAML::Node dynamicsConfig; 
        if(!ConfigManager::get().getConfig<YAML::Node>("Dynamics", dynamicsConfig, ConfigManager::get().getFullConfig()))
        {
            std::cerr << "Invalid or missing Dynamics configuration" << std::endl; 
            return nullptr; 
        }

        std::string dynamicsType; 
        if(!ConfigManager::get().getConfig<std::string>("type", dynamicsType, dynamicsConfig))
        {
            std::cerr << "Invalid or missing dynamics type" << std::endl; 
            return nullptr; 
        }

        std::string dynamicsConfigFile; 
        if(!ConfigManager::get().getConfig<std::string>("file", dynamicsConfigFile, dynamicsConfig))
        {
            std::cerr << "Invalid or missing dynamics config file" << std::endl; 
            return nullptr; 
        }

        std::string fullpath = aFullConfigFilePath + dynamicsConfigFile; 
        YAML::Node dynamicsSpecificConfig; 
        if(!ConfigManager::get().load(fullpath, dynamicsSpecificConfig))
        {
            std::cerr << "Invalid or missing dynamics specific config file" << std::endl; 
            return nullptr; 
        }

        return create(dynamicsType, dynamicsSpecificConfig); 
    }

    static inline std::shared_ptr<IDynamicSystem> create(const std::string& aDynamicsType, const YAML::Node& aDynamicsSpecificConfig)
    {
        if("linear" == aDynamicsType || "Linear" == aDynamicsType || "stateSpace" == aDynamicsType)
        {
            // load linear system
            return std::make_shared<LinearSystem>(); 
        }
        else
        {
            std::cerr << "Unsupported dynamics type: " << aDynamicsType << std::endl; 
            return nullptr; 
        }
    }

private:
   
};
#endif //DYNAMICSYSTEMFACTORY_HPP