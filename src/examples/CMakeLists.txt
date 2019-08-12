#---------------------------------------
# Find 3rd-party libraries
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

#---------------------------------------
# Shared sources for all examples
file(GLOB shared_srcs "*.cpp" "*.h")

#---------------------------------------
# Compile examples
add_subdirectory(fireworks)