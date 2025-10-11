#ifndef UTILS_HPP
#define UTILS_HPP

#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>

namespace utils
{
    inline Eigen::MatrixXd eigenMatrixFromConfig(const YAML::Node& aMatrixInYaml)
    {   
        size_t rows = aMatrixInYaml.size();
        size_t cols = aMatrixInYaml[0].size();

        Eigen::MatrixXd matrix(rows, cols);

        for (size_t i = 0; i < rows; ++i) 
        {
            const YAML::Node& row = aMatrixInYaml[i];
            
            for (size_t j = 0; j < cols; ++j) 
            {
                matrix(i, j) = row[j].as<double>();
            }
        }

        return matrix; 
    }

    inline Eigen::VectorXd eigenVectorFromConfig(const YAML::Node& aVectorInYaml)
    {
        size_t size = aVectorInYaml.size();

        Eigen::VectorXd vec(size); 

        for (size_t i = 0; i < size; ++i) 
        {
            vec(i) = aVectorInYaml[i].as<double>();
        }

        return vec; 
    }

} // namespace utils
#endif