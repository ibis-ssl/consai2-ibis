cmake_minimum_required(VERSION 2.8.3)

add_subdirectory(behavior_tree)
add_subdirectory(play)
add_subdirectory(tactic)
add_subdirectory(world_model)
add_subdirectory(utils)
add_subdirectory(field_analysis)

set(root_sources
        play_executer.cpp
        app/app.cpp)

add_library(root ${root_sources})
target_link_libraries(root
        field_analysis
        play_test
        imgui
        glad
        glfw
        -L/opt/ros/melodic/lib
        -lroscpp_serialization
        ${catkin_LIBRARIES}
        -lroscpp_serialization)

