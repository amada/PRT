#include <stdio.h>
#include <cmath>

#include "vecmath.h"
#include "triangle.h"
#include "camera.h"
#include "ray.h"
#include "bvh.h"
#include "image.h"
#include "diffuse_visualizer.h"


namespace prt
{

void DiffuseVisualizer::TraceBlock(Image& image, uint32_t x0, uint32_t y0, uint32_t x1, uint32_t y1, const Scene& scene, const Camera& camera)
{
    for (uint32_t y = y0; y <= y1; y++) {
        for (uint32_t x = x0; x <= x1; x++) {
            Vector3f color = Trace(camera, scene, x, y);

            image.WritePixel(x, y, color);
        }
    }
}

float repeat(float f)
{
    return f - std::floor(f);
}

Vector3f DiffuseVisualizer::Trace(const Camera& camera, const Scene& scene, uint32_t x, uint32_t y)
{
    Vector3f color(0);

    auto ray = camera.GenerateJitteredRay(m_rand, x, y);
    RayIntersection intr;
    scene.intersect(intr, ray);

    if (intr.isHit()) {
        SurfaceProperties prop;
        scene.getSurfaceProperties(prop, intr);

        color = prop.material->sampleDiffuse(prop.uv);
    }

    return color;
}

} // namespace prt
