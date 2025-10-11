
#include "gpc/ControlSystem.h"
#include "gpc/RateController.hpp"
#include <iostream>
#include <thread> 

ControlSystem::ControlSystem(std::shared_ptr<IStateFetcher> aStateFetcher) : 
    mStateFetcher(aStateFetcher), mControlRate(-1), mDynamicSystem(nullptr), 
    mUseSafetyFilter(false)
{

}

ControlSystem::~ControlSystem()
{

}

void ControlSystem::init(const std::string& aFilePath, const std::string& aFileName)
{
    // load the full config file 
    if(!ConfigManager::get().load(aFilePath + aFileName))
    {
        throw std::runtime_error("Invalid or missing configuration"); 
    } 

    // create the controller impl 
    mController = ControllerFactory::create(aFilePath);  
    if(nullptr == mController) throw std::runtime_error("Invalid Controller configuration"); 
    
    if(mController->isModelBased())
    {
        mDynamicSystem = DynamicSystemFactory::create(aFilePath);
        if(nullptr == mDynamicSystem) throw std::runtime_error("Invalid Dynamics configuration");
    }
    
    YAML::Node controlSystemConfig; 
    if(!ConfigManager::get().getConfig<YAML::Node>("ControlSystem", controlSystemConfig))
    {
        throw std::runtime_error("Missing or invalid ControlSystem configuration"); 
    }

    if(!ConfigManager::get().getConfig<int>("rate", mControlRate, controlSystemConfig))
    {
        throw std::runtime_error("Missing or invalid ControlSystem rate"); 
    }

    if(!ConfigManager::get().getConfig<bool>("safetyFilterEnabled", mUseSafetyFilter, controlSystemConfig))
    {
        std::string safetyFilterState = mUseSafetyFilter == true ? "enabled" : "disabled"; 
        std::cout << "Safety Filter " + safetyFilterState << std::endl;
    }

    if(mUseSafetyFilter)
    {
        mSafetyFilter = SafetyFilterFactory::create(aFilePath); 
        if(nullptr == mSafetyFilter) throw std::runtime_error("Invalid SafetyFilter configuration"); 
    }

    if(!mController->init(mDynamicSystem))
    {
        std::cerr << "Failed to initialize the controller" << std::endl; 
        return; 
    }
}

void ControlSystem::run()
{
    std::vector<double> values = {1.0, 2.0, 3.0};
    Eigen::VectorXd goal = Eigen::Map<Eigen::VectorXd>(values.data(), values.size());
    
    RateController rate(mControlRate); 
    rate.start(); 
    rate.block(); // control rate warm-up  

    while(true)
    {
        rate.start(); 

        auto state = mStateFetcher->fetchState(); 
        Eigen::VectorXd controlInput = mController->compute(goal, state, rate.getDeltaTime());  

        if(mSafetyFilter && mUseSafetyFilter)
        {
            mSafetyFilter->compute(controlInput); 
        }

        //mCommander->send(controlInput); 
        
        rate.block(); 
    }
}
