prt - Playground for Ray Tracing
================================

This is a playground repository for ray tracing-related experiments. The implementation is done from scratch, using CPU, and referencing various resources, especially [Physically Based Rendering From Theory to Implementation](https://www.pbr-book.org/).

Features
--------
* Path tracing
* Acceleration structures
  * BVH for triangle mesh
* Support for SIMD instructions
  * Arm Neon, AVX2
  * Using SIMD lanes for ray packet-like tracing


Supported platforms
-------------------
The code is developed primarily on macOS, but should work on other platforms.
* macOS
* Windows
* Raspberry Pi


How to build
------------
TODO


Rendered image
--------------
![Cornell Box + Teapot](images/cornell_box_teapot.png)