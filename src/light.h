#pragma once

#include "vecmath.h"
#include "material.h"

namespace prt
{

enum class LightType : uint32_t {
    kDirectional,
    kInfiniteArea
};

class Light
{
public:
};

struct DirectionalLight
{
    void init() { dir = 0.0f; intensity = 0.0f; }

    Vector3f dir;
    Vector3f intensity;
};

class InfiniteAreaLight
{
public:
    InfiniteAreaLight() = default;
    void init();
    void create(const char* path);
    void release();
    void sample(Vector3f& dir, Vector3f& intensity, const Vector2f& u) const;

    bool isValid() const { return m_image.isValid(); }

private:
    Texture m_image;
    float* m_verticalP;
    float* m_horizontalP;
    // CDF size
    int32_t m_width;
    int32_t m_height;
};


} // namespace prt
