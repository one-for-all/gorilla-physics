get_filename_component(demo ${CMAKE_CURRENT_LIST_DIR} NAME)

project(${demo})

file(GLOB srcs "*.cpp" "*.h")
set(required_libraries gorilla)

add_executable(${demo} ${srcs} ${shared_srcs})
target_link_libraries(${demo} PRIVATE ${required_libraries})

# Link OpenGL and GLUT
target_include_directories(${demo} PRIVATE ${OPENGL_INCLUDE_DIRS}
                           ${GLUT_INCLUDE_DIRS})
target_link_libraries(${demo} PRIVATE ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})