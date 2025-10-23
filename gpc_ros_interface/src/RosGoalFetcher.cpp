
#include "RosGoalFetcher.h"
#include "RosTopicManager.hpp"

RosGoalFetcher::RosGoalFetcher()
{
    RosTopicManager::getInstance()->createSubscriber<robot_idl::msg::GpcGoal>("gpc/goal", 
                                                                              std::bind(&RosGoalFetcher::goalCallback, 
                                                                                        this, 
                                                                                        std::placeholders::_1)); 
    //mGoal = Eigen::Matrix<double, 12, 1>::Zero(); 
}

RosGoalFetcher::~RosGoalFetcher()
{
    
}

Eigen::VectorXd RosGoalFetcher::fetchGoal()
{
    std::lock_guard<std::mutex> lock(mGoalMutex); 
    return mGoal; 
}

void RosGoalFetcher::setGoal(Eigen::VectorXd aGoal)
{
    std::lock_guard<std::mutex> lock(mGoalMutex); 
    mGoal = aGoal;
}

void RosGoalFetcher::goalCallback(robot_idl::msg::GpcGoal::SharedPtr aMsg)
{   
    using namespace robot_idl::msg; 
    Eigen::VectorXd goal;
    std::vector<double> ref; 

    switch (aMsg->mode)
    {
    case GpcGoal::MODE_SETPOINT:
        
        ref = aMsg->x_ref;
        goal.resize(ref.size()); 
    
        for(int i = 0; i < ref.size(); i++)
        {
            goal(i) = ref[i]; 
        }   

        setGoal(goal); 

        break;
    
    default:
        break;
    }
}