#pragma once

#include "vecmath.h"

namespace prt
{

class Random;
class Ray;
class SoaRay;

class Camera
{
public:
    Camera(const Vector3f& pos, const Vector3f& dir, uint32_t width, uint32_t height)
     : m_pos(pos), m_dir(dir), m_width(width), m_height(height)
    {
        m_invWidth = 1.0f/width;
        m_invHeight = 1.0f/height;
    }

    Ray GenerateJitteredRay(Random& rand, uint32_t x, uint32_t y) const;
    SoaRay GenerateJitteredSoaRay(Random& rand, uint32_t x, uint32_t y) const;

    Vector3f m_pos;
    Vector3f m_dir;
    uint32_t m_width;
    uint32_t m_height;
    float m_invWidth;
    float m_invHeight;
};

} // namespace prt