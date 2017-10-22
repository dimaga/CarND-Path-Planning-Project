project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources
    src/Trajectory.cpp
    src/Trajectory.h
    src/ITrajectory.h
    src/Obstacles.cpp
    src/Obstacles.h
    src/Planner.cpp
    src/Planner.h
    src/IObstacles.h
    src/Costs.cpp
    src/Costs.h
    src/ICosts.h
    src/Map.cpp
    src/Map.h
    src/IMap.h
    src/Waypoint.h)

set(sources_main
    src/main.cpp)

set(sources_unittests
    src/MapTests.cpp
    src/TrajectoryTests.cpp
    src/ObstaclesTests.cpp
    src/CostsTests.cpp
    src/PlannerTests.cpp
    src/unittests.cpp)

set(headers)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

add_executable(path_planning ${sources} ${sources_main} ${headers})
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR})

target_link_libraries(path_planning z ssl uv uWS)

add_executable(unit_test_path_planning ${sources} ${sources_unittests} ${headers})
