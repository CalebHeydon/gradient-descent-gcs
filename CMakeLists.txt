cmake_minimum_required(VERSION 3.12)
project(gcs)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 11)

include_directories("include")

file(GLOB SOURCES "src/*.c")
add_library(gcs STATIC ${SOURCES})
target_link_libraries(gcs)

file(GLOB TEST_MEMORY_SOURCES "tests/test_memory.c")
add_executable(test_memory ${TEST_MEMORY_SOURCES})
target_link_libraries(test_memory gcs)
