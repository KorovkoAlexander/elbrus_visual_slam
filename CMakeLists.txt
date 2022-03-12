cmake_minimum_required(VERSION 3.10)
project(tensorrt_models)

set(CMAKE_CXX_STANDARD 14)

if (UNIX)
    set(CMAKE_CXX_FLAGS "-fPIC -O3")
else()
    set(CMAKE_CXX_FLAGS "-O3")
endif (UNIX)

file(GLOB srcSources src/*.cpp)

include_directories(${PROJECT_SOURCE_DIR}/include)
add_subdirectory(pybind11)

execute_process(
        COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
message( STATUS "Architecture: ${ARCHITECTURE}" )

# Elbrus
include_directories(${PROJECT_SOURCE_DIR}/elbrus/include)
add_library(elbrus SHARED IMPORTED)
if( ${ARCHITECTURE} STREQUAL "x86_64" )
    set_property(TARGET elbrus PROPERTY IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/elbrus/lib_x86_64/libelbrus.so)
elseif( ${ARCHITECTURE} STREQUAL "aarch64" )
    set_property(TARGET elbrus PROPERTY IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/elbrus/lib_aarch64_xavier/libelbrus.so)
endif()

pybind11_add_module(elbrus_visual_slam MODULE ${srcSources})
target_link_libraries(elbrus_visual_slam PRIVATE elbrus)

set_target_properties(elbrus_visual_slam
        PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}" SUFFIX "${PYTHON_MODULE_EXTENSION}")