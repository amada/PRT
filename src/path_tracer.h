#pragma once

#include "stats.h"
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
    PathTracer() { m_stats.clear(); }

    void TraceBlock(Image& image, uint32_t x0, uint32_t y0, uint32_t x1, uint32_t y1, const Scene& scene, const Camera& camera, uint32_t samples);
    Vector3f Trace(const Camera& camera, const Scene& scene, uint32_t x, uint32_t y, uint32_t samples);
    Stats getStats() const { return m_stats; }

private:
    Vector3f TracePacket(const Camera& camera, const Scene& scene, uint32_t x, uint32_t y, uint32_t samples, const TraverseStackCache* stackCache);

    Vector3f ComputeRadiance(
        const Scene& scene,
        const Vector3f& rayDir, const Vector3f& pos, const SurfaceProperties& prop);

    Vector3f ComputeRadiance(
        const Scene& scene, const RayHitPacket& hitPacket, const RayPacket& packet);

    Random m_rand;

    // debug
    Stats m_stats;
    //bool m_verbose = false;
};

} // namespace prt