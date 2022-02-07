#pragma once

#include "vecmath.h"
#include "random.h"

namespace prt
{

class Scene;
struct Material;
class Image;
class Camera;

class PathTracer
{
public:
    PathTracer() {}

    void TraceBlock(Image& image, uint32_t x0, uint32_t y0, uint32_t x1, uint32_t y1, const Scene& scene, const Camera& camera, uint32_t samples);
    Vector3f Trace(const Camera& camera, const Scene& scene, uint32_t x, uint32_t y, uint32_t samples);

private:
    Vector3f SoaTrace(const Camera& camera, const Scene& scene, uint32_t x, uint32_t y, uint32_t samples);

    Vector3f ComputeRadiance(
        const Scene& scene,
        const Vector3f& rayDir, const Vector3f& pos, const Vector3f& normal, const Material* material, uint32_t depth);

    Random m_rand;

    // debug
    bool m_verbose = false;
};

} // namespace prt