#include <stdio.h>
#include <cmath>

#include "vecmath.h"
#include "triangle.h"
#include "camera.h"
#include "ray.h"
#include "bvh.h"
#include "scene.h"
#include "image.h"
#include "gbuffer_visualizer.h"


namespace prt
{

void GbufferVisualizer::TraceBlock(Image& image, uint32_t x0, uint32_t y0, uint32_t x1, uint32_t y1, const Scene& scene, const Camera& camera)
{
    for (uint32_t y = y0; y <= y1; y++) {
        for (uint32_t x = x0; x <= x1; x++) {
            Vector3f color = Trace(camera, scene, x, y);

            image.writePixel(x, y, color);
        }
    }
}

Vector3f GbufferVisualizer::Trace(const Camera& camera, const Scene& scene, uint32_t x, uint32_t y)
{
    Vector3f color(0);

    auto ray = camera.GenerateJitteredRay(m_rand, x, y);
    SingleRayHitPacket hitPacket;
    scene.intersect(hitPacket, SingleRayPacket(ray));

    const auto& hit = hitPacket.hit;
    if (hit.isHit()) {
        SurfaceProperties prop;
        scene.getSurfaceProperties(prop, hit);

        switch (m_type) {
        case Type::kDiffuse: color = prop.material->sampleDiffuse(prop.uv); break;
        case Type::kMeshNormal: color = prop.material->sampleBump(prop)*0.5f + 0.5f; break;
        case Type::kNormal: color = prop.material->sampleBump(prop)*0.5f + 0.5f; break;
        default:
            PRT_ASSERT(false);
        }
    }

    return color;
}

} // namespace prt
