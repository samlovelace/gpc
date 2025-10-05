
#include "gpc/ControlSystem.h"
#include "gpc/RateController.hpp"
#include <iostream>
#include <thread> 

ControlSystem::ControlSystem(std::shared_ptr<IStateFetcher> aStateFetcher) : mStateFetcher(aStateFetcher), mControlRate(-1)
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

    if(nullptr == mController)
    {
        throw std::runtime_error("Invalid controller configuration"); 
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


}

void ControlSystem::run()
{
    std::vector<double> values = {1.0, 2.0, 3.0};
    Eigen::VectorXd goal = Eigen::Map<Eigen::VectorXd>(values.data(), values.size());
    
    RateController controlRate(mControlRate); 
    std::this_thread::sleep_for(std::chrono::duration<double>(1/(double)mControlRate)); 

    while(true)
    {
        controlRate.start(); 

        auto state = mStateFetcher->fetchState(); 
        mController->compute(goal, state, controlRate.getDeltaTime());  
        
        controlRate.block(); 
    }
}
