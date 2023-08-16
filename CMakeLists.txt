cmake_minimum_required(VERSION 3.16)
project(QEM)

# Release type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Libigl
include(libigl)

# CGAL
find_package(CGAL REQUIRED)

# Enable the target igl::glfw
igl_include(glfw)

# Include folder in ./include
include_directories(${CMAKE_SOURCE_DIR}/include/igl)

# Add your project files
file(GLOB SRC_FILES src/*.cpp)

# Previously commented out executable; Uncomment if needed.
# add_executable(${PROJECT_NAME} ${SRC_FILES} src/dataparser.h src/dataparser.cpp src/mesh.cpp src/mesh.h)

# Your current executable
add_executable(${PROJECT_NAME} ${SRC_FILES} src/quadratic.h src/collapse_callback.h src/helper.h src/process.h src/manifold.h src/preprocess.h)

# Linking both igl::glfw and CGAL to your project
target_link_libraries(${PROJECT_NAME} PUBLIC igl::glfw CGAL::CGAL)
