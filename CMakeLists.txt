cmake_minimum_required(VERSION 3.1)

#set( CMAKE_VERBOSE_MAKEFILE on )

if(WIN32)
    set(CMAKE_GENERATOR_TOOLSET "ClangCL")
endif()

project(prt CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -flto=thin -Wall -Werror")
if(CMAKE_HOST_SYSTEM_PROCESSOR STREQUAL "arm64")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mavx2")
endif()
add_link_options("-lz")

add_subdirectory(src)
add_subdirectory(tests)
