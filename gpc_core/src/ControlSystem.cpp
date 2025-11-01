
#include "gpc/ControlSystem.h"
#include "gpc/RateController.hpp"
#include <iostream>
#include <thread> 

#include "gpc/EigenPrinter.hpp"

ControlSystem::ControlSystem(std::shared_ptr<IStateFetcher> aStateFetcher, 
                             std::shared_ptr<IGoalFetcher> aGoalFetcher, 
                             std::shared_ptr<IActuatorCommander> aCommander) : 
    mStateFetcher(aStateFetcher), 
    mGoalFetcher(aGoalFetcher),
    mCommander(aCommander),
    mControlRate(-1), 
    mDynamicSystem(nullptr), 
    mUseSafetyFilter(false), 
    mGuidance(std::make_shared<GuidanceSystem>())
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
        mSafetyFilter->configure(12); // TODO:  
    }

    if(!mController->init(mDynamicSystem))
    {
        std::cerr << "Failed to initialize the controller" << std::endl; 
        return; 
    }

    // setup goal fetcher interface 
    mGoalFetcher->bindGuidance(mGuidance); 
    if(!mGoalFetcher->startListening())
    {
        std::cerr << "Failed to initialize GoalFetcher listening thread" << std::endl; 
        return; 
    }
}

void ControlSystem::run()
{   
    auto state = mStateFetcher->fetchState(); 
    mGuidance->setGoal(state); 

    RateController rate(mControlRate); 
    rate.start(); 
    rate.block(); // control rate warm-up  

    Obstacle obs; 
    obs.pos = Eigen::Vector3d(1, 1, 0); 
    obs.vel = Eigen::Vector3d::Zero();
    obs.acc = Eigen::Vector3d::Zero(); 
    obs.radius = 0.25; 

    Obstacle other; 
    other.pos = Eigen::Vector3d(0, 1.75, 0); 
    other.vel = Eigen::Vector3d::Zero();
    other.acc = Eigen::Vector3d::Zero(); 
    other.radius = 0.25; 

    while (true) 
    {
        rate.start();

        Eigen::VectorXd state = mStateFetcher->fetchState(); 
        Eigen::VectorXd goal = mGuidance->getNextGoal(state); 
        
        EigenPrinter singleState(EigenPrinter::Style::SingleLine, 4, "State: ");   
        singleState.print(state);
        
        Eigen::VectorXd controlInput = mController->compute(goal, state, rate.getDeltaTime());

        if (mUseSafetyFilter && mSafetyFilter) 
        {
            SafetyContext sctx;
            sctx.x     = state;                 
            sctx.xdot  = Eigen::VectorXd();     // optional
            sctx.u_nom = controlInput;
            sctx.dt    = rate.getDeltaTime();
            sctx.radius = 0.25; // TODO: dont hardcode 
            sctx.obstacles.push_back(obs); 
            sctx.obstacles.push_back(other);  

            Eigen::VectorXd u_safe;
            if (mSafetyFilter->compute(sctx, u_safe)) 
            {
                controlInput = u_safe;
                EigenPrinter single(EigenPrinter::Style::SingleLine, 4, "Safe Control Input: ");   
                single.print(controlInput);
            } 
            else 
            {
                // TODO: 
                // fallback policy if infeasible (clip, zero, last valid, etc.)
                // controlInput.setZero();
            }
        }
        
        mCommander->send(controlInput);

        rate.block();
    }

}
