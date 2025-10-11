#ifndef CONTROLSYSTEM_H
#define CONTROLSYSTEM_H
 
#include <memory>
#include "gpc/IStateFetcher.hpp"
#include "gpc/ControllerFactory.hpp"
#include "gpc/DynamicSystemFactory.hpp"
#include "gpc/SafetyFilterFactory.hpp"
#include "gpc/ConfigManager.hpp"
 
class ControlSystem 
{ 
public:
    ControlSystem(std::shared_ptr<IStateFetcher> aStateFetcher); 
    ~ControlSystem();

    void init(const std::string& aFilePath, const std::string& aFileName); 
    void run(); 

private:
    std::shared_ptr<IStateFetcher> mStateFetcher; 
    std::shared_ptr<IController> mController; 
    std::shared_ptr<IDynamicSystem> mDynamicSystem; 
    std::shared_ptr<ISafetyFilter> mSafetyFilter; 

    int mControlRate; 
    bool mUseSafetyFilter; 

};
#endif //CONTROLSYSTEM_H