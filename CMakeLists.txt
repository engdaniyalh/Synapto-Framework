cmake_minimum_required(VERSION 3.20)
project(SynaptoFramework VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Optimization & Real-Time Hardening Flags
add_compile_options(-Wall -Wextra -Wpedantic -O3 -fno-omit-frame-pointer)

find_package(Threads REQUIRED)

include(FetchContent)

# 1. open62541 (OPC UA Server / Client)
FetchContent_Declare(
    open62541
    GIT_REPOSITORY https://github.com/open62541/open62541.git
    GIT_TAG v1.3.8
)
set(UA_ENABLE_AMALGAMATION ON CACHE BOOL "" FORCE)
set(UA_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(UA_BUILD_UNIT_TESTS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(open62541)

# 2. GoogleTest for Unit & Integration Testing
enable_testing()
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_subdirectory(src/reflex_engine)
add_subdirectory(tests/test_reflex_engine)