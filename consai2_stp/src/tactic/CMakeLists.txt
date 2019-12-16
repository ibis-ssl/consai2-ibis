set(tactic_sources
        kick_to_target.cpp
        pass_receive.cpp
        )

add_library(tactic ${tactic_sources})
target_link_libraries(tactic skill)
add_subdirectory(skill)