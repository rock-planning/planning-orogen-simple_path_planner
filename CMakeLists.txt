PROJECT(simple_path_planner)
cmake_minimum_required(VERSION 2.6)

add_definitions( -DEIGEN_DONT_VECTORIZE -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)

SET (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/.orogen/config")
INCLUDE(simple_path_plannerBase)

# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)

