
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
    std::vector<double> values = {3.0, 2.0, 1.0};
    Eigen::VectorXd state = Eigen::Map<Eigen::VectorXd>(values.data(), values.size()); 
    return state;
}