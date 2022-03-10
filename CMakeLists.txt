cmake_minimum_required(VERSION 3.10)
project(tensorrt_models)

set(CMAKE_CXX_STANDARD 14)

if (UNIX)
    set(CMAKE_CXX_FLAGS "-fPIC -O3")
else()
    set(CMAKE_CXX_FLAGS "-O3")
endif (UNIX)

include_directories(${PROJECT_SOURCE_DIR}/elbrus/include)
link_directories(${PROJECT_SOURCE_DIR}/elbrus/lib_x86_64)

file(GLOB srcSources src/*.cpp)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_subdirectory(pybind11)
#add_subdirectory(spdlog)

add_library(elbrus_visual_slam MODULE ${srcSources})

target_link_libraries(elbrus_visual_slam
        pybind11::module
#        spdlog::spdlog
        )

set_target_properties(elbrus_visual_slam PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
        SUFFIX "${PYTHON_MODULE_EXTENSION}")