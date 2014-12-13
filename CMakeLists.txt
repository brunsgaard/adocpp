cmake_minimum_required (VERSION 2.8.8)
project (ado)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")

# Surpress warnings from xtaci/algorithms
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-language-extension-token -Wno-unused-function")

# Set a default build type if none was specified (we are not using an IDE)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Debug' as none was specified.")
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build." FORCE)
endif()

# Add src before libs to make use it don't use installed headers
include_directories(src)

# Add https://github.com/xtaci/algorithms
include_directories(algorithms-master/include)
include_directories(algorithms-master)

set(link_libs
)

add_subdirectory (src)

