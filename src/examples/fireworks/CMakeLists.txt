#---------------------------------------------
# Find current example name
get_filename_component(target ${CMAKE_CURRENT_LIST_DIR} NAME)

project(${target})

#----------------------------------------------
# Compile example
file(GLOB srcs "*.cpp" "*.h")
set(required_libraries gorilla)

add_executable(${target} ${srcs} ${shared_srcs})
target_link_libraries(${target} PRIVATE ${required_libraries})

#-----------------------------------------------
# Link with OpenGL and GLUT
target_include_directories(${target} PRIVATE ${OPENGL_INCLUDE_DIRS}
                           ${GLUT_INCLUDE_DIRS})
target_link_libraries(${target} PRIVATE ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})