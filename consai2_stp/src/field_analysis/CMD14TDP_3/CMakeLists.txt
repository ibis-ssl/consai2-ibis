cmake_minimum_required(VERSION 2.8.3)

set(condition_sources
        receive_condition.cpp
        shoot_condition.cpp
        )

add_library(condition STATIC ${condition_sources})
target_link_libraries(condition world_model)
