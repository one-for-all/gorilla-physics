cmake_minimum_required (VERSION 3.10)
project(gorilla)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -fPIC")

set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR})

file(GLOB sources
    "src/*"
)

add_library(gorilla SHARED
    ${sources}
)
target_include_directories(gorilla PUBLIC
    include)

# Build demos
add_subdirectory(src/demos demos)
