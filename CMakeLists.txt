cmake_minimum_required(VERSION 3.0)
project(ceres_bundle_adjustment)

add_subdirectory(pybind11)
# Add Ceres Solver dependency
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

pybind11_add_module(ceres_bundle_adjustment main.cpp)
target_link_libraries(ceres_bundle_adjustment PRIVATE ${CERES_LIBRARIES})