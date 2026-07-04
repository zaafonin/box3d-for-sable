cmake_minimum_required(VERSION 3.22)

project(sable_box3d_native LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(BOX3D_STATIC_CXX_RUNTIME "Statically link libstdc++/libgcc for Linux distribution builds when the toolchain provides them." OFF)

if(NOT BOX3D_SOURCE_DIR)
    set(BOX3D_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/box3d")
endif()

if(NOT EXISTS "${BOX3D_SOURCE_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "BOX3D_SOURCE_DIR must point to a Box3D source checkout. Missing: ${BOX3D_SOURCE_DIR}/CMakeLists.txt")
endif()

set(BOX3D_SAMPLES OFF CACHE BOOL "" FORCE)
set(BOX3D_BENCHMARKS OFF CACHE BOOL "" FORCE)
set(BOX3D_DOCS OFF CACHE BOOL "" FORCE)
set(BOX3D_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(BOX3D_VALIDATE OFF CACHE BOOL "" FORCE)
set(BOX3D_DOUBLE_PRECISION ON CACHE BOOL "" FORCE)

find_package(JNI REQUIRED COMPONENTS JVM)

add_subdirectory("${BOX3D_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/box3d")

add_library(sable_box3d SHARED
    sable_box3d_jni.cpp
)

target_include_directories(sable_box3d
    PRIVATE
        ${JNI_INCLUDE_DIRS}
)

target_link_libraries(sable_box3d
    PRIVATE
        box3d::box3d
)

if(BOX3D_STATIC_CXX_RUNTIME AND CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_link_options(sable_box3d PRIVATE -static-libstdc++ -static-libgcc)
endif()
