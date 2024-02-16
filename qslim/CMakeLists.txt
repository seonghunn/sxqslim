cmake_minimum_required(VERSION 3.16)
project(sxqslim)

# Release type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# libigl
add_subdirectory(external/libigl)
include_directories(${CMAKE_SOURCE_DIR}/external/libigl/include)
# CGAL
find_package(CGAL REQUIRED)

# Enable the target igl::glfw
igl_include(glfw)

# Include folder in ./include
#include_directories(${CMAKE_SOURCE_DIR}/include/igl)
include_directories(${CMAKE_SOURCE_DIR}/src)
#include_directories(${CMAKE_SOURCE_DIR}/include)

# Add your project files
file(GLOB SRC_FILES src/*.cpp)

# get pybind
FetchContent_Declare(
        pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11.git
        GIT_TAG        v2.6.1  # pybind11의 특정 버전을 지정
)
FetchContent_MakeAvailable(pybind11)

# Define the Python module
pybind11_add_module(${PROJECT_NAME} MODULE ${SRC_FILES} src/pybind.cpp)

# Linking both igl::glfw, CGAL, and Pybind11 to your project
target_link_libraries(${PROJECT_NAME} PRIVATE igl::glfw CGAL::CGAL pybind11::module)
