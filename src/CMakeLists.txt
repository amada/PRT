project(prt_lib)

set(prt_lib_headers bvh.h camera.h gbuffer_visualizer.h image.h light.h log.h material.h mesh.h path_tracer.h platform.h prt.h random.h ray.h sample_models.h scene.h stats.h thread_pool.h triangle.h vecmath.h texture.h)
set(prt_lib_sources bvh.cpp triangle.cpp vecmath.cpp thread_pool.cpp image.cpp scene.cpp mesh.cpp path_tracer.cpp camera.cpp gbuffer_visualizer.cpp material.cpp light.cpp log.cpp sample_models.cpp texture.cpp)
set(prt_lib_ext_sources ../ext/tinyexr/deps/miniz/miniz.c)
add_library(prt_lib ${prt_lib_sources} ${prt_lib_ext_sources} ${prt_lib_headers})
target_include_directories(prt_lib PRIVATE ../ext/tinyexr/deps/miniz)

project(prt_main CXX)
add_executable(prt_main main.cpp)
target_link_libraries(prt_main prt_lib)
