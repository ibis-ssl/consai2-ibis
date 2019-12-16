cmake_minimum_required(VERSION 2.8.3)

set(world_model_sources
        robot_node.cpp
        tool.cpp
        world_model.cpp
        assignment.cpp)

add_library(world_model STATIC ${world_model_sources})
target_link_libraries(world_model behavior_tree field_analysis)