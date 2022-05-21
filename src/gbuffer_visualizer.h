#pragma once

#include "vecmath.h"
#include "random.h"

namespace prt
{

class Scene;
class Image;
class Camera;

class GbufferVisualizer
{
public:
    enum class Type {
        kDiffuse,
        kMeshNormal,
        kNormal
    };

    GbufferVisualizer(Type type = Type::kDiffuse) : m_type(type) {}

    void TraceBlock(Image& image, uint32_t x0, uint32_t y0, uint32_t x1, uint32_t y1, const Scene& scene, const Camera& camera);
    Vector3f Trace(const Camera& camera, const Scene& scene, uint32_t x, uint32_t y);

private:
    Type m_type;
    Random m_rand;
};

} // namespace prt