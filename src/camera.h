#pragma once

#include "vecmath.h"

namespace prt
{

class Random;
class Ray;
struct RayPacket;
//class SoaRay;

class Camera
{
public:
    Camera(const Vector3f& pos, const Vector3f& dir, uint32_t width, uint32_t height)
     : m_pos(pos), m_dir(normalize(dir)), m_width(width), m_height(height)
    {
        m_invWidth = 1.0f/width;
        m_invHeight = 1.0f/height;

        auto up = Vector3f(0, 1.0f, 0);
        auto right = cross(dir, up);
        if (length(right) < 0.00001f) {
            right = cross(dir, Vector3f(1, 0, 0));
        }
        right = normalize(right);
        up = normalize(cross(right, dir));

        m_up = up;
        m_right = right;
    }

    Ray GenerateJitteredRay(Random& rand, uint32_t x, uint32_t y) const;
    RayPacket GenerateJitteredRayPacket(Random& rand, uint32_t x, uint32_t y) const;
//    SoaRay GenerateJitteredSoaRay(Random& rand, uint32_t x, uint32_t y) const;

    Vector3f m_pos;
    Vector3f m_dir;
    Vector3f m_up;
    Vector3f m_right;
    uint32_t m_width;
    uint32_t m_height;
    float m_invWidth;
    float m_invHeight;
};

} // namespace prt