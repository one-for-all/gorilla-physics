cmake_minimum_required (VERSION 3.3)
project(gorilla)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR})

file(GLOB sources
    "src/*"    
)
include_directories(include)
add_library(gorilla SHARED
    ${sources}
)