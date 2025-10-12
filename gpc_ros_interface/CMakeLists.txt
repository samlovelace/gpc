cmake_minimum_required(VERSION 3.14)
project(gpc_ros_interface)

find_package(rclcpp REQUIRED)
find_package(robot_idl REQUIRED) 

# Add the gpc library as a subdirectory
add_subdirectory(../gpc_core gpc_core)

set(INTERFACE_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/src/RosStateFetcher.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/RosGoalFetcher.cpp
    main.cpp
)

add_executable(${PROJECT_NAME} ${INTERFACE_SOURCES})

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  robot_idl
)

# Link against gpc
target_link_libraries(${PROJECT_NAME} gpc)

# Include gpc headers
target_include_directories(${PROJECT_NAME} PRIVATE
    ../gpc_core/include
    ${CMAKE_CURRENT_LIST_DIR}/include
)

# Install
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY ../configuration/ DESTINATION share/${PROJECT_NAME}/configuration)

ament_package()