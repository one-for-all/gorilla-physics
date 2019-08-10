get_filename_component(demo ${CMAKE_CURRENT_LIST_DIR} NAME)

project(${demo})

file(GLOB srcs "*.cpp" "*.h" "../*.cpp" "../*.h")
set(required_libraries gorilla)

message(${srcs})
add_executable(${demo} ${srcs})
target_link_libraries(${demo} PUBLIC ${required_libraries})

# Find OpenGL and GLUT
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
target_include_directories(${demo} PUBLIC ${OPENGL_INCLUDE_DIRS}
                           ${GLUT_INCLUDE_DIRS})
target_link_libraries(${demo} PUBLIC ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})