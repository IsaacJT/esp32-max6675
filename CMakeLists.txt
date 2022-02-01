cmake_minimum_required (VERSION 3.5)

file(GLOB_RECURSE SOURCE_FILES src/*.c)

idf_component_register(
    SRCS ${SOURCE_FILES}
    INCLUDE_DIRS "include"
)
