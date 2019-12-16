cmake_minimum_required(VERSION 2.8.3)

set(utils_sources
        control_target_builder.cpp
        target.cpp
        )

add_library(utils ${utils_sources})
target_link_libraries(utils world_model)
