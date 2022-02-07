project(prt_lib CXX)
add_library(prt_lib bvh.cpp triangle.cpp vecmath.cpp main.cpp thread_pool.cpp image.cpp mesh.cpp path_tracer.cpp camera.cpp sample_models.cpp)


project(prt_main CXX)
add_executable(prt_main main.cpp)
target_link_libraries(prt_main prt_lib)
