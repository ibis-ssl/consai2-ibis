cmake_minimum_required(VERSION 2.8.3)

set(test_sources
        test_pass.cpp
        test_kick_to_target.cpp
        test_goalie.cpp
        test_ball_placement.cpp
        )

add_library(play_test ${test_sources})
target_link_libraries(play_test tactic)