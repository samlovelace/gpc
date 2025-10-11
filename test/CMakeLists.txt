cmake_minimum_required(VERSION 3.10)
project(test_gpc)

find_package(GTest REQUIRED)
find_package(OsqpEigen REQUIRED)

set(GPC_CORE_INCLUDE_DIR 
    ${CMAKE_CURRENT_LIST_DIR}/../gpc_core/include
)

set(TEST_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/math/QpSolver_Test.cpp
)

set(CORE_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/../gpc_core/src/QpSolver.cpp
)

add_executable(${PROJECT_NAME} test.cpp ${TEST_SOURCES} ${CORE_SOURCES})
target_include_directories(${PROJECT_NAME} PRIVATE 
    ${GPC_CORE_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME} GTest::GTest OsqpEigen::OsqpEigen)