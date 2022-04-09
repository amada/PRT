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
        Vector3f normal;

        auto bvh = m_bvh[hit.meshId];
        auto& mesh = bvh->m_mesh;

        uint32_t indexBase = hit.primId*Mesh::kVertexCountPerPrim;
        uint32_t v0 = mesh.getIndex(indexBase + 0);
        uint32_t v1 = mesh.getIndex(indexBase + 1);
        uint32_t v2 = mesh.getIndex(indexBase + 2);

        if (bvh->m_mesh.hasVertexNormal()) {
            const auto &n0 = mesh.getNormal(v0);
            const auto &n1 = mesh.getNormal(v1);
            const auto &n2 = mesh.getNormal(v2);
            normal = normalize(hit.i * n0 + hit.j * n1 + hit.k * n2);
        } else {
            const Vector3f &p0 = mesh.getPosition(v0);
            const Vector3f &p1 = mesh.getPosition(v1);
            const Vector3f &p2 = mesh.getPosition(v2);
            normal = normalize(cross(p1 - p0, p2 - p0));
        }

// TODO: remove indirection?
// use 3 floats at once
        v0 = mesh.getTexcoordIndex(indexBase + 0);
        v1 = mesh.getTexcoordIndex(indexBase + 1);
        v2 = mesh.getTexcoordIndex(indexBase + 2);
        #if 1
        const auto& t0 = mesh.getTexcoord(v0);
        const auto& t1 = mesh.getTexcoord(v1);
        const auto& t2 = mesh.getTexcoord(v2);
        properties.uv = hit.i*t0 + hit.j*t1 + hit.k*t2;
        #endif

        properties.normal = normal;
        properties.material = &mesh.getMaterial(mesh.getPrimToMaterial(hit.primId));
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