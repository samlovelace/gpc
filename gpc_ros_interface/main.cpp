
#include "gpc.h"
#include "RosStateFetcher.h"
#include "RosGoalFetcher.h"
#include "RosTopicManager.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>


int main()
{
    //std::signal(SIGINT, signalHandler); 
    rclcpp::init(0, nullptr);
    RosTopicManager::getInstance()->spinNode(); 

    std::string configFilePath = ament_index_cpp::get_package_share_directory("gpc_ros_interface") + "/configuration/"; 

    auto stateFetcher = std::make_shared<RosStateFetcher>(); 
    auto goalFetcher = std::make_shared<RosGoalFetcher>(); 

    ControlSystem cs(stateFetcher, goalFetcher);
    cs.init(configFilePath, "config.yaml");  
    cs.run(); 

    rclcpp::shutdown(); 
}