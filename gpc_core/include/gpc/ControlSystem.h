#ifndef CONTROLSYSTEM_H
#define CONTROLSYSTEM_H
 
#include <memory>
#include "gpc/IStateFetcher.hpp"
#include "gpc/IGoalFetcher.hpp"
#include "gpc/IActuatorCommander.hpp"
#include "gpc/ControllerFactory.hpp"
#include "gpc/DynamicSystemFactory.hpp"
#include "gpc/SafetyFilterFactory.hpp"
#include "gpc/ConfigManager.hpp"

#include "gpc/GuidanceSystem.h"
 
class ControlSystem 
{ 
public:
    ControlSystem(std::shared_ptr<IStateFetcher> aStateFetcher, 
                  std::shared_ptr<IGoalFetcher> aGoalFetcher, 
                  std::shared_ptr<IActuatorCommander> aCommander);
                  
    ~ControlSystem();

    void init(const std::string& aFilePath, const std::string& aFileName); 
    void run(); 

private:
    std::shared_ptr<IStateFetcher> mStateFetcher; 
    std::shared_ptr<IGoalFetcher> mGoalFetcher; 
    std::shared_ptr<IController> mController; 
    std::shared_ptr<IDynamicSystem> mDynamicSystem; 
    std::shared_ptr<ISafetyFilter> mSafetyFilter; 
    std::shared_ptr<IActuatorCommander> mCommander;
    
    std::shared_ptr<GuidanceSystem> mGuidance; 

    int mControlRate; 
    bool mUseSafetyFilter; 

};
#endif //CONTROLSYSTEM_H