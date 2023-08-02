cmake_minimum_required(VERSION 3.16)
project(QEM)

list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)


# Libigl
include(libigl)

# CGAL
#find_package(CGAL REQUIRED)
#include_directories(${CGAL_INCLUDE_DIRS})

# Enable the target igl::glfw
igl_include(glfw)

# Include folder in ./include
# include_directories(${CMAKE_SOURCE_DIR}/include)

# Add your project files
file(GLOB SRC_FILES src/*.cpp)

#add_executable(${PROJECT_NAME} ${SRC_FILES} src/dataparser.h src/dataparser.cpp src/mesh.cpp src/mesh.h)
add_executable(${PROJECT_NAME} ${SRC_FILES} src/quadratic.h src/collapse_callback.h src/helper.h src/process.h src/manifold.h src/preprocess.h)
target_link_libraries(${PROJECT_NAME} PUBLIC igl::glfw)
