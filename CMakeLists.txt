cmake_minimum_required(VERSION 3.12)
project(gcs)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 11)

file(GLOB SOURCES "src/*.c")
add_library(gcs STATIC ${SOURCES})
target_include_directories(gcs PRIVATE "include")
target_link_libraries(gcs)

file(GLOB TEST_GCS_SOURCES "tests/test_gcs.c")
add_executable(test_gcs ${TEST_GCS_SOURCES})
target_include_directories(test_gcs PRIVATE "include")

if(WIN32)
    set(MATH "")
else()
    set(MATH "m")
endif()

target_link_libraries(test_gcs gcs ${MATH})
