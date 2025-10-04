
#include "gpc/ControlSystem.h"
#include <iostream>
#include <thread> 

ControlSystem::ControlSystem(std::shared_ptr<IStateFetcher> aStateFetcher) : mStateFetcher(aStateFetcher)
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

}

void ControlSystem::run()
{
    Eigen::VectorXd goal(3); 

    while(true)
    {
        auto state = mStateFetcher->fetchState(); 
        mController->compute(goal, state);  
        std::this_thread::sleep_for(std::chrono::seconds(1)); 
    }
}
