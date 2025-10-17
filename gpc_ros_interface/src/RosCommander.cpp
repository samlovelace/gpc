
#include "RosCommander.h"
#include "RosTopicManager.hpp"
#include "robot_idl/msg/abv_command.hpp"

RosCommander::RosCommander()
{
    RosTopicManager::getInstance()->createPublisher<robot_idl::msg::AbvCommand>("abv/command"); 

}

RosCommander::~RosCommander()
{

}

void RosCommander::send(Eigen::VectorXd aControlInput)
{
    robot_idl::msg::AbvVec3 data; 
    data.set__x(aControlInput[0]); 
    data.set__y(aControlInput[1]); 
    data.set__yaw(aControlInput[6]); 

    robot_idl::msg::AbvCommand cmd; 
    cmd.set__type("thruster"); 
    cmd.set__data(data); 

    RosTopicManager::getInstance()->publishMessage<robot_idl::msg::AbvCommand>("abv/command", cmd); 
}