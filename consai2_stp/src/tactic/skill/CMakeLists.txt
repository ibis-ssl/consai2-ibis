cmake_minimum_required(VERSION 2.8.3)

set(skill_sources
        face_to_target.cpp
        go_between.cpp
        go_point.cpp
        spin_at_target.cpp
)

add_library(skill ${skill_sources})
target_link_libraries(skill utils)