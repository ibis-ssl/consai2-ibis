cmake_minimum_required(VERSION 2.8.3)

add_subdirectory(CMD14TDP_3)

set(field_analysis_sources
        field_analyzer.cpp
        )

add_library(field_analysis STATIC ${field_analysis_sources})
target_link_libraries(field_analysis condition)

