cmake_minimum_required(VERSION 3.10)

project(test_gpc)

find_package(GTest REQUIRED)
add_executable(${PROJECT_NAME} test.cpp)
target_link_libraries(${PROJECT_NAME} GTest::GTest)