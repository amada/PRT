cmake_minimum_required(VERSION 3.1)

#set( CMAKE_VERBOSE_MAKEFILE on )

if(WIN32)
    set(CMAKE_GENERATOR_TOOLSET "ClangCL")
endif()

project(prt CXX)

set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -mavx2")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -flto=thin -mavx2 -Wall -Werror")

add_subdirectory(src)
add_subdirectory(tests)
