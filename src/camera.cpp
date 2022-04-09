#include <stdio.h>
#include <cmath>

#include "vecmath.h"
#include "random.h"
#include "ray.h"
#include "camera.h"

namespace prt
{

Ray Camera::GenerateJitteredRay(Random& rand, uint32_t x, uint32_t y) const
{
    Ray ray;
    const float screenScale = 0.6f;

    float s = 0.125f*m_invWidth;
    float dx = rand.generateMinus1to1() * s;
    float dy = rand.generateMinus1to1() * s;
    float nx = 2.0f*(x*m_invWidth - 0.5f + dx)*screenScale;
    float ny = -2.0f*(y*m_invHeight - 0.5f + dy)*screenScale;
    auto d = normalize(nx*m_right + ny*m_up + m_dir);

    ray.maxT = 100000.0f;
    ray.org = m_pos;
    ray.dir = d;

    ray.prepare();

    return ray;
}

RayPacket Camera::GenerateJitteredRayPacket(Random& rand, uint32_t x, uint32_t y) const
{
    RayPacket packet;

    Vector3f avgDir(0.0f);

    for (uint32_t v = 0; v < RayPacket::kVectorCount; v++) {
        SoaRay ray;
        const float screenScale = 0.6f;
        float s = 0.125f*m_invWidth;

        Vector3f dir[SoaConstants::kLaneCount];

        for (uint32_t i = 0; i < SoaConstants::kLaneCount; i++) {
            float dx = rand.generateMinus1to1() * s;
            float dy = rand.generateMinus1to1() * s;
            float nx = 2.0f*(x*m_invWidth - 0.5f + dx)*screenScale;
            float ny = -2.0f*(y*m_invHeight - 0.5f + dy)*screenScale;
            dir[i] = normalize(nx*m_right + ny*m_up + m_dir);
            avgDir = avgDir + dir[i];
        }

        ray.maxT = 100000.0f;
        ray.org = m_pos;
        auto d = SoaVector3f(dir);
        ray.dir = d;

        ray.prepare();

        packet.rays[v] = ray;
    }

    packet.avgDir = avgDir/RayPacket::kSize;

    return packet;
}

} // namespace prt