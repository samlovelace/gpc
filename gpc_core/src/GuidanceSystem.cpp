
#include "gpc/GuidanceSystem.h"
#include "gpc/RateController.hpp"

GuidanceSystem::GuidanceSystem() : mCurrentMode(MODE::SETPOINT), mCenterPointFound(false) 
{

}

GuidanceSystem::~GuidanceSystem()
{

}

void GuidanceSystem::setGoal(const Eigen::VectorXd& aGoal)
{
    std::lock_guard<std::mutex> lock(mGoalMutex); 
    mGoal = aGoal; 
}

void GuidanceSystem::setMode(const MODE& aMode)
{
    std::lock_guard<std::mutex> lock(mGoalMutex);
    mCurrentMode = aMode; 
}

void GuidanceSystem::setTrajectory(const TRAJECTORY& aTrajectory)
{
    std::lock_guard<std::mutex> lock(mGoalMutex);
    mCurrentTrajectory = aTrajectory; 

    // reset vars for trajectory back to default 
    mCenterPointFound = false; 
}

Eigen::VectorXd GuidanceSystem::getGoal()
{
    std::lock_guard<std::mutex> lock(mGoalMutex); 
    return mGoal; 
}

Eigen::VectorXd GuidanceSystem::getNextGoal(const Eigen::VectorXd& aCurrentState)
{
    Eigen::VectorXd nextGoal; 

    switch (mCurrentMode)
    {
        case MODE::SETPOINT:
            nextGoal = getGoal(); 
            break;

        case MODE::TRAJECTORY:
            nextGoal = stepTrajectory(aCurrentState); 
            break; 
        
        default:
            break;
    }

    return nextGoal; 
}

Eigen::VectorXd GuidanceSystem::stepTrajectory(const Eigen::VectorXd& aCurrentState)
{
    Eigen::VectorXd nextGoal; 

    switch (mCurrentTrajectory)
    {
        case TRAJECTORY::LINE_2D:
        {
            nextGoal = line2d(aCurrentState); 
            break;
        }
        case TRAJECTORY::CIRCLE_2D:
        {
            double x_c, y_c; 
            double radius = 0.4;
            bool clockwise = true; 
            double fractionStep = 0.1; 

            if(!mCenterPointFound)
            {
                // first time this traj is being invoked, 
                // determine center point of circle
                double x0 = aCurrentState[0]; 
                double y0 = aCurrentState[1];
                double yaw = aCurrentState[6];  

                if (clockwise) {
                    x_c = x0 + radius * std::sin(yaw);
                    y_c = y0 - radius * std::cos(yaw);
                } else { // CCW
                    x_c = x0 - radius * std::sin(yaw);
                    y_c = y0 + radius * std::cos(yaw);
                }
                
                mCenterPointFound = true; 
            }

            nextGoal = circle2d(aCurrentState, x_c, y_c, clockwise, fractionStep, radius); 
            break; 
        }
        default:
            break;
    }

    return nextGoal; 
}

Eigen::VectorXd GuidanceSystem::circle2d(const Eigen::VectorXd& aCurrentState, 
                                         const double& aCenterX, 
                                         const double aCenterY, 
                                         bool aClockwise, 
                                         const double& aFractionStep, 
                                         const double& aRadius)
{
    // TODO: obtain somehow from user what plane circle should be on, 
    // Options: xy, xz, yz
    // user can also choose whether constant orientation or maintain pointing towards center 

    double x = aCurrentState[0]; 
    double y = aCurrentState[1]; 
    double yaw = aCurrentState[6]; 

    // Compute current angle around circle
    double theta_curr = std::atan2(y - aCenterY, 
                                   x - aCenterX);

    // Compute angle increment (percentage of 2π)
    double dir = aClockwise ? -1.0 : 1.0;
    double dTheta = dir * (2.0 * M_PI * aFractionStep);

    double theta_next = theta_curr + dTheta;

    // Compute next goal position
    double x_goal = aCenterX + aRadius * std::cos(theta_next);
    double y_goal = aCenterY + aRadius * std::sin(theta_next);

    // Heading toward the center
    double yaw_goal = std::atan2(aCenterX - y_goal, aCenterY - x_goal);

    Eigen::VectorXd nextGoal(aCurrentState.size()); 
    nextGoal.setZero(); 

    nextGoal[0] = x_goal; 
    nextGoal[1] = y_goal; 
    nextGoal[6] = yaw_goal; 

    return nextGoal; 
}

Eigen::VectorXd GuidanceSystem::line2d(const Eigen::VectorXd& aCurrentState)
{

}