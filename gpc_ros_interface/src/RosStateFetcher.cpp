
#include "RosStateFetcher.h"
#include <iostream>
#include "RosTopicManager.hpp"

RosStateFetcher::RosStateFetcher() 
{
    Eigen::VectorXd init = Eigen::Vector3d::Zero(); 
    setLatestState(init); 

    RosTopicManager::getInstance()->createSubscriber<robot_idl::msg::AbvState>("abv/state", 
                                                                               std::bind(&RosStateFetcher::stateCallback,
                                                                                        this, 
                                                                                        std::placeholders::_1)); 
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
    // TODO: this is the proper way to populate the state 
    // Eigen::VectorXd state(13); 
    // state[0] = aMsg->position.x; 
    // state[1] = aMsg->position.y; 
    // state[2] = 0.0; // constant z for abv state  
    // state[3] = aMsg->velocity.x; 
    // state[4] = aMsg->velocity.y;
    // state[5] = 0.0; // constant z vel for abv state 
    
    // state[6] = aMsg->position.yaw; 
    // state[7] = 0.0; 
    // state[8] = 0.0; 
    // state[9] = aMsg->velocity.yaw; 
    // state[10] = 0.0; 
    // state[11] = 0.0; 


    Eigen::VectorXd state(3); 
    state[0] = aMsg->position.x; 
    state[1] = aMsg->position.y; 
    state[2] = aMsg->position.yaw; 
    
    setLatestState(state); 
}