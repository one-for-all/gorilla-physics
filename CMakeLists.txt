cmake_minimum_required (VERSION 3.10)
project(gorilla)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -fPIC")

file(GLOB sources
    "src/*"
)

add_library(gorilla SHARED
    ${sources}
)

target_include_directories(gorilla PUBLIC
    include
)

# Build demos
add_subdirectory(src/demos demos)
