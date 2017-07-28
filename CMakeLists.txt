project(path_planning)
cmake_minimum_required(VERSION 3.5)
add_definitions(-std=c++11)
set(CXX_FLAGS, "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

aux_source_directory(./src SRC)

add_executable(path_planning ${SRC})
target_link_libraries(path_planning z ssl uv uWS)
