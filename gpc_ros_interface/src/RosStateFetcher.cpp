
#include "RosStateFetcher.h"
#include <iostream>
#include "RosTopicManager.hpp"
#include <thread>

RosStateFetcher::RosStateFetcher() : mFirstStateCallback(false)
{
    RosTopicManager::getInstance()->createSubscriber<robot_idl::msg::AbvState>("abv/state", 
                                                                               std::bind(&RosStateFetcher::stateCallback,
                                                                                        this, 
                                                                                        std::placeholders::_1)); 

    while(!mFirstStateCallback)
    {
        std::cout << "Waiting for state...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1)); 
    }
}

RosStateFetcher::~RosStateFetcher()
{

}

Eigen::VectorXd RosStateFetcher::fetchState()
{
    std::lock_guard<std::mutex> lock(mStateMutex); 
    return mLatestState;
}

void RosStateFetcher::setLatestState(Eigen::VectorXd aState)
{
    std::lock_guard<std::mutex> lock(mStateMutex); 
    mLatestState = aState; 
}

void RosStateFetcher::stateCallback(robot_idl::msg::AbvState::SharedPtr aMsg)
{ 
    mFirstStateCallback = true; 

    Eigen::VectorXd state(12); 
    state[0] = aMsg->position.x; 
    state[1] = aMsg->position.y; 
    state[2] = aMsg->position.z;   
    state[3] = aMsg->velocity.x; 
    state[4] = aMsg->velocity.y;
    state[5] = aMsg->velocity.z;  
    
    state[6] = aMsg->orientation.z; 
    state[7] = aMsg->orientation.y; 
    state[8] = aMsg->orientation.x; 
    state[9] = aMsg->ang_vel.z; 
    state[10] = aMsg->ang_vel.y; 
    state[11] = aMsg->ang_vel.x; 
    
    setLatestState(state); 
}