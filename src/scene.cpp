#include "log.h"
#include "vecmath.h"
#include "ray.h"
#include "scene.h"


namespace prt
{

void Scene::init()
{
    m_directionalLight = {0.0, 0.0};
}

template<typename T, typename U>
void Scene::getSurfaceProperties(T& properties, const U& hit) const
{
    if constexpr (std::is_same<U, RayHit>::value) {
        auto bvh = m_bvh[hit.meshId];
        auto& mesh = bvh->m_mesh;

        mesh.getSurfaceProperties(properties, hit);
     } else if constexpr (std::is_same<U, SoaRayHit>::value) {
        __builtin_trap();
    } else {
        __builtin_trap();
    }
}

template void Scene::getSurfaceProperties<SurfaceProperties, RayHit>(SurfaceProperties&, const RayHit&) const;

template<typename T, typename R>
void Scene::intersect(T& hitPacket, const R& packet) const
{
    memset(&hitPacket, 0, sizeof(hitPacket));

    // Find intersection closer than maxT
    hitPacket.setMaxT(packet);

    // TODO bbox/as for bvhs
    for (auto &bvh : m_bvh) {
        // TODO ray transform per BVH

        bvh->intersect(hitPacket, packet);
    }

    hitPacket.setMissForMaxT(packet);
}

template void Scene::intersect<SingleRayHitPacket, SingleRayPacket>(SingleRayHitPacket&, const SingleRayPacket&) const;
template void Scene::intersect<RayHitPacket, RayPacket>(RayHitPacket&, const RayPacket&) const;


template<typename T, typename R>
T Scene::occluded(const R& ray) const
{
    // TODO bbox/as for bvhs
    for (auto &bvh : m_bvh) {
        // TODO ray transform per BVH

        if (bvh->occluded<T, R>(ray))
            return true;
    }

    return false;
}

template bool Scene::occluded<bool, Ray>(const Ray&) const;


} // namespace prt