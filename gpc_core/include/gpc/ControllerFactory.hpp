#ifndef CONTROLLERFACTORY_H
#define CONTROLLERFACTORY_H
 
#include <string> 
#include <memory>
#include "gpc/ConfigManager.hpp"
#include "gpc/IController.hpp"

#include "gpc/PIDController.h"
#include "gpc/LQRController.h"
 
class ControllerFactory 
{ 
public:
    ControllerFactory();
    ~ControllerFactory();

    static inline std::shared_ptr<IController> create(const std::string& aFullConfigFilePath)
    {
        YAML::Node controllerConfig; 
        if(!ConfigManager::get().getConfig<YAML::Node>("Controller", controllerConfig, ConfigManager::get().getFullConfig()))
        {
            std::cerr << "Invalid or missing Controller configuration" << std::endl; 
            return nullptr; 
        }

        std::string controllerType; 
        if(!ConfigManager::get().getConfig<std::string>("type", controllerType, controllerConfig))
        {
            std::cerr << "Invalid or missing Controller type" << std::endl; 
            return nullptr; 
        }

        std::string controllerConfigFile; 
        if(!ConfigManager::get().getConfig<std::string>("file", controllerConfigFile, controllerConfig))
        {
            std::cerr << "Invalid or missing Controller config file" << std::endl; 
            return nullptr; 
        }

        std::string fullpath = aFullConfigFilePath + controllerConfigFile; 
        YAML::Node controllerSpecificConfig; 
        if(!ConfigManager::get().load(fullpath, controllerSpecificConfig))
        {
            std::cerr << "Invalid or missing Controller specific config file" << std::endl; 
            return nullptr; 
        }

        return create(controllerType, controllerSpecificConfig); 
    }

    static inline std::shared_ptr<IController> create(const std::string& aControllerType, const YAML::Node& aControllerSpecificConfig)
    {
        if("PID" == aControllerType || "pid" == aControllerType)
        {
            // parse configured gains 
            std::map<std::string, std::vector<double>> gains; 
            gains["Kp"] = aControllerSpecificConfig["gains"]["Kp"].as<std::vector<double>>(); 
            gains["Ki"] = aControllerSpecificConfig["gains"]["Ki"].as<std::vector<double>>();
            gains["Kd"] = aControllerSpecificConfig["gains"]["Kd"].as<std::vector<double>>();

            std::vector<int> indices = aControllerSpecificConfig["indices"].as<std::vector<int>>(); 

            return std::make_shared<PIDController>(gains, indices);
        }
        else if ("LQR" == aControllerType || "lqr" == aControllerType)
        {
            return std::make_shared<LQRController>(); 
        }
        else
        {
            std::cerr << "Unsupported controller type: " << aControllerType << std::endl; 
            return nullptr; 
        }
    }


private:
   
};
#endif //CONTROLLERFACTORY_H