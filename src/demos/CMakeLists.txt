find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

file(GLOB shared_srcs "*.cpp" "*.h")

add_subdirectory(fireworks)