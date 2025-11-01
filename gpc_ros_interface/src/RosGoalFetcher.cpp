
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

bool RosGoalFetcher::startListening()
{
    // ensure ros comms are good 
    while(!rclcpp::ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(150)); 
    }

    return true; 
}

void RosGoalFetcher::goalCallback(robot_idl::msg::GpcGoal::SharedPtr aMsg)
{   
    using namespace robot_idl::msg; 
    Eigen::VectorXd goal;
    std::vector<double> ref; 

    switch (aMsg->mode)
    {
    case GpcGoal::MODE_SETPOINT:
    {
        ref = aMsg->x_ref;
        goal.resize(ref.size()); 
    
        for(int i = 0; i < ref.size(); i++) {
            goal(i) = ref[i]; 
        }   

        mGuidance->setMode(GuidanceSystem::MODE::SETPOINT); 
        mGuidance->setGoal(goal); 
        break;
    }
    case GpcGoal::MODE_TRAJECTORY:
    {
        // will probably end up switching on trajectory types 
        mGuidance->setMode(GuidanceSystem::MODE::TRAJECTORY); 
        auto trajectoryToSet = GuidanceSystem::TRAJECTORY::NUM_TYPES; 

        switch (aMsg->trajectory_type)
        {
        case GpcGoal::TRAJ_LINE2D:
            trajectoryToSet = GuidanceSystem::TRAJECTORY::LINE_2D; 
            break;
        case GpcGoal::TRAJ_CIRCLE2D: 
            trajectoryToSet = GuidanceSystem::TRAJECTORY::CIRCLE_2D; 
        default:
            break;
        }

        // get type of traj from message 
        mGuidance->setTrajectory(trajectoryToSet);
    }

    
    default:
        break;
    }
}