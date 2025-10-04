
#include "gpc/ControlSystem.h"
#include "gpc/RateController.hpp"
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
    int rate = 1; // Hz TODO: make configurable 
    RateController controlRate(rate); 
    std::this_thread::sleep_for(std::chrono::duration<double>(1/(double)rate)); 

    while(true)
    {
        controlRate.start(); 

        auto state = mStateFetcher->fetchState(); 
        mController->compute(goal, state, controlRate.getDeltaTime());  
        
        controlRate.block(); 
    }
}
