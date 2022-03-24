#pragma once

#include "vecmath.h"
#include "random.h"

namespace prt
{

class Scene;
class Image;
class Camera;

class DiffuseVisualizer
{
public:
    DiffuseVisualizer() {}

    void TraceBlock(Image& image, uint32_t x0, uint32_t y0, uint32_t x1, uint32_t y1, const Scene& scene, const Camera& camera);
    Vector3f Trace(const Camera& camera, const Scene& scene, uint32_t x, uint32_t y);

private:
    Random m_rand;
};

} // namespace prt