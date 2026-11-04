cmake_minimum_required(VERSION 3.16)
project(gt_mods CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(GT_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../gran-turismo-decomp/include" CACHE PATH "gran-turismo-decomp include dir (holds gt_mod/gt_mod_api.h)")

enable_testing()
add_subdirectory(analog-pedals)
