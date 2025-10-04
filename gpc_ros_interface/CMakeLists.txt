cmake_minimum_required(VERSION 3.14)
project(gpc_ros_interface)

# Add the gpc library as a subdirectory
add_subdirectory(../gpc_core gpc_core)

set(INTERFACE_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/src/RosStateFetcher.cpp
    main.cpp
)

add_executable(${PROJECT_NAME} ${INTERFACE_SOURCES})

# Link against gpc
target_link_libraries(${PROJECT_NAME} PRIVATE gpc)

# Include gpc headers
target_include_directories(${PROJECT_NAME} PRIVATE
    ../gpc_core/include
    ${CMAKE_CURRENT_LIST_DIR}/include
)
