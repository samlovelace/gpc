
#include "RosStateFetcher.h"
#include <iostream>

RosStateFetcher::RosStateFetcher()
{

}

RosStateFetcher::~RosStateFetcher()
{

}

Eigen::VectorXd RosStateFetcher::fetchState()
{
    std::cout << "Fetching state from ros topic..." << std::endl; 
    return Eigen::VectorXd(3); 
}