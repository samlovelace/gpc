#ifndef GUIDANCESYSTEM_H
#define GUIDANCESYSTEM_H

#include <mutex> 
#include <Eigen/Dense>
 
class GuidanceSystem 
{ 
public:
    GuidanceSystem();
    ~GuidanceSystem();

    enum class MODE
    {
        SETPOINT, 
        TRAJECTORY, 
        NUM_TYPES
    };

    enum class TRAJECTORY
    {
        LINE_2D,
        CIRCLE_2D, 
        NUM_TYPES
    }; 

    void setGoal(const Eigen::VectorXd& aGoal); 
    void setMode(const MODE& aMode); 
    void setTrajectory(const TRAJECTORY& aTrajectory); 
    Eigen::VectorXd getNextGoal(const Eigen::VectorXd& aCurrentState); 

private: 
    Eigen::VectorXd getGoal(); 
    Eigen::VectorXd stepTrajectory(const Eigen::VectorXd& aCurrentState);

    // Trajectory implementations 
    Eigen::VectorXd line2d(const Eigen::VectorXd& aCurrentState); 
    
    Eigen::VectorXd circle2d(const Eigen::VectorXd& aCurrentState, 
                             const double& aCenterX, 
                             const double aCenterY, 
                             bool aClockwise, 
                             const double& aFractionStep, 
                             const double& aRadius); 

private:

    MODE mCurrentMode;
    TRAJECTORY mCurrentTrajectory; 
    Eigen::VectorXd mGoal; 
    std::mutex mGoalMutex;  

    bool mCenterPointFound; 
   
};
#endif //GUIDANCESYSTEM_H   