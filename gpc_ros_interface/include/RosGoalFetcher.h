#ifndef ROSGOALFETCHER_H
#define ROSGOALFETCHER_H

#include "gpc/IGoalFetcher.hpp"
#include "robot_idl/msg/gpc_goal.hpp"
#include <mutex>

class RosGoalFetcher : public IGoalFetcher
{ 
public:
    RosGoalFetcher();
    ~RosGoalFetcher() override; 

    Eigen::VectorXd fetchGoal() override; 
    void setGoal(Eigen::VectorXd aGoal) override; 

private:
    std::mutex mGoalMutex; 

private: 
    void goalCallback(const robot_idl::msg::GpcGoal::SharedPtr aMsg); 

};
#endif //ROSGOALFETCHER_H