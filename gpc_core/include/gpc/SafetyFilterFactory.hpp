#ifndef SAFETYFILTERFACTORY_HPP
#define SAFETYFILTERFACTORY_HPP
 
#include <string> 
#include <memory>
#include "gpc/ConfigManager.hpp"
#include "gpc/ConstraintFactory.hpp"
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
            std::cout << "Making QP based safety filter" << std::endl;
            std::cout << YAML::Dump(aSafetyFilterSpecificConfig) << std::endl;  
            
            // parse config file for constraints 
            std::shared_ptr<IConstraint> c = nullptr; 
            std::vector<std::shared_ptr<IConstraint>> constraints; 

            for(const auto& constraintConfig : aSafetyFilterSpecificConfig["Constraints"])
            {
                std::string type = constraintConfig["type"].as<std::string>(); 
                std::cout << "Making constraint of type: " << type << std::endl; 

                c = ConstraintFactory::create(type, constraintConfig); 
                if(nullptr == c) throw std::runtime_error("Invalid constraint configuration"); 
                constraints.push_back(c); 
            }

            Eigen::VectorXd weights = utils::eigenVectorFromConfig(aSafetyFilterSpecificConfig["Weights"]); 

            auto sf = std::make_shared<QpSafetyFilter>(weights); 
            sf->setConstraints(constraints); 
            return sf;  
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