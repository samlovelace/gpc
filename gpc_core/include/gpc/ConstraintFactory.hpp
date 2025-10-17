#ifndef CONSTRAINTFACTORY_HPP
#define CONSTRAINTFACTORY_HPP
 
#include <memory>
#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Dense>
#include "gpc/utils.hpp"
#include "gpc/IConstraint.hpp" 
#include <iostream>

#include "gpc/InputBoxConstraint.h"
#include "gpc/EcbfDistanceConstraint.h"

class ConstraintFactory 
{ 
public:
    ConstraintFactory();
    ~ConstraintFactory();

    static inline std::shared_ptr<IConstraint> create(const std::string& aType, const YAML::Node& aConstraintsConfig)
    {
        if("input_box" == aType)
        {
            Eigen::VectorXd umin = utils::eigenVectorFromConfig(aConstraintsConfig["min"]); 
            Eigen::VectorXd umax = utils::eigenVectorFromConfig(aConstraintsConfig["max"]); 

            return std::make_shared<InputBoxConstraint>(umin, umax); 
        }
        else if ("ecbf_distance" == aType)
        {
            return std::make_shared<EcbfDistanceConstraint>(); 
        }
        else
        {
            std::cout << "Unsupported constraint type: " << aType << std::endl;
            return nullptr; 
        }

        return nullptr; 
    }


private:
   
};
#endif //CONSTRAINTFACTORY_HPP