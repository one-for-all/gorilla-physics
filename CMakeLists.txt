cmake_minimum_required (VERSION 3.10)
project(gorilla)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -fPIC")

#---------------------------------------------
# Check if use single precision for floats
if(USE_SINGLE_PRECISION)
    add_definitions(-DSINGLE_PRECISION)
endif(USE_SINGLE_PRECISION)

#---------------------------------------------
# Compile gorilla physics engine
file(GLOB sources
    "src/*")

add_library(gorilla SHARED
    ${sources})

target_include_directories(gorilla PUBLIC
    include)

#--------------------------------------------
# Compile examples
add_subdirectory(src/examples examples)
