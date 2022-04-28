#include "log.h"
#include "vecmath.h"
#include "ray.h"
#include "scene.h"


namespace prt
{

void Scene::init()
{
    m_directionalLight.init();
    m_infiniteAreaLight.init();
    m_availableLights = 0;
    m_bbox = BBox::init();
    m_radius = std::numeric_limits<float>::max();
}

void Scene::add(Bvh* bvh) 
{ 
    bvh->m_mesh.m_id = m_bvh.size();
    m_bvh.push_back(bvh);
    m_bbox.grow(bvh->m_mesh.getBBox());

    auto center = m_bbox.center();
    m_radius = length(m_bbox.upper - center);
} // shared ptr?


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
void Scene::intersect(T& hitPacket, const R& packet, const TraverseStackCache* stackCache) const
{
    memset(&hitPacket, 0, sizeof(hitPacket));

    // Find intersection closer than maxT
    hitPacket.setMaxT(packet);

    // TODO bbox/as for bvhs
    for (auto &bvh : m_bvh) {
        // TODO ray transform per BVH

        bvh->intersect(hitPacket, packet, stackCache);
    }

    hitPacket.setMissForMaxT(packet);
}

template void Scene::intersect<SingleRayHitPacket, SingleRayPacket>(SingleRayHitPacket&, const SingleRayPacket&, const TraverseStackCache*) const;
template void Scene::intersect<RayHitPacket, RayPacket>(RayHitPacket&, const RayPacket&, const TraverseStackCache*) const;


template<typename T, typename R>
T Scene::occluded(const RayPacketMask& _mask, const R& packet) const
{
    auto mask = _mask;
    // TODO bbox/as for bvhs
    for (auto &bvh : m_bvh) {
        // TODO ray transform per BVH

        auto res = bvh->occluded<T, R>(mask, packet);
        if constexpr (std::is_same<R, SingleRayPacket>::value) {
            if (res)
                return res;
        } else if constexpr (std::is_same<R, RayPacket>::value) {
            mask = mask | res;
            if (mask.allTrue())
                return mask;
        }

    }

    if constexpr (std::is_same<R, SingleRayPacket>::value) {
        return false;
    } else if constexpr (std::is_same<R, RayPacket>::value) {
        return mask;
    }
}

template bool Scene::occluded<bool, SingleRayPacket>(const RayPacketMask& mask, const SingleRayPacket&) const;
template RayPacketMask Scene::occluded<RayPacketMask, RayPacket>(const RayPacketMask& mask, const RayPacket&) const;


} // namespace prt