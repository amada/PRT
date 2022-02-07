project(prt_tests CXX)

add_executable(prt_tests tests.cpp)
include_directories(${CMAKE_SOURCE_DIR}/src)
target_link_libraries(prt_tests prt_lib)
