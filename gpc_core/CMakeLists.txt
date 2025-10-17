cmake_minimum_required(VERSION 3.1)
project(gpc)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED) 
find_package(OsqpEigen REQUIRED)

set(PROJECT_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/src/ControlSystem.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/EcbfDistanceConstraint.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/InputBoxConstraint.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/LinearSystem.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/PIDController.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/QpSafetyFilter.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/QpSolver.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/LQRController.cpp
)

add_library(${PROJECT_NAME} STATIC ${PROJECT_SOURCES})

target_link_libraries(gpc PUBLIC Eigen3::Eigen yaml-cpp OsqpEigen::OsqpEigen)

target_include_directories(${PROJECT_NAME} PUBLIC 
    ${CMAKE_CURRENT_LIST_DIR}/include
)

