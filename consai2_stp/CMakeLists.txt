cmake_minimum_required(VERSION 2.8.3)
project(consai2_stp)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_compile_options(-std=gnu++17)

#set(CMAKE_MODULE_PATH cotire/CMake)
#include(cotire/CMake/cotire.cmake)
include(3rdparty/3rdparty.cmake)
include(3rdparty/config.cmake)

find_package(OpenMP REQUIRED)
if (OpenMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif ()

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        tf
        consai2_msgs
        )

catkin_package(
        INCLUDE_DIRS include
        #  LIBRARIES consai2_game
        #  CATKIN_DEPENDS roscpp rospy std_msgs
        #  DEPENDS system_lib
)

###########
## Build ##
###########

add_compile_options(-Ofast -march=native -Wall)
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        ./include/
        ${catkin_INCLUDE_DIRS}
        /usr/include/eigen3
        ./closest_point
        ${PYTHON_INCLUDE_DIRS}
)

add_subdirectory(src)

add_executable(consai2_stp_node src/consai2_stp_node.cpp)
add_executable(test_pass_node src/test_pass.cpp)
add_executable(test_ball_placement_node src/test_ball_placement.cpp)
add_executable(test_goalie_node src/test_goalie.cpp)

add_dependencies(consai2_stp_node consai2_msgs_generate_messages_cpp)
add_dependencies(test_pass_node consai2_msgs_generate_messages_cpp)
add_dependencies(test_ball_placement_node consai2_msgs_generate_messages_cpp)
add_dependencies(test_goalie_node consai2_msgs_generate_messages_cpp)

target_link_libraries(consai2_stp_node root)

target_link_libraries(test_pass_node root)

target_link_libraries(test_ball_placement_node root)

target_link_libraries(test_goalie_node root)

