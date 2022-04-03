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
void Scene::getSurfaceProperties(T& properties, const U& intr) const
{
    if constexpr (std::is_same<U, RayIntersection>::value) {
        Vector3f normal;

        auto bvh = m_bvh[intr.meshId];
        auto& mesh = bvh->m_mesh;

        uint32_t indexBase = intr.primId*Mesh::kVertexCountPerPrim;
        uint32_t v0 = mesh.getIndex(indexBase + 0);
        uint32_t v1 = mesh.getIndex(indexBase + 1);
        uint32_t v2 = mesh.getIndex(indexBase + 2);

        if (bvh->m_mesh.hasVertexNormal()) {
            const auto &n0 = mesh.getNormal(v0);
            const auto &n1 = mesh.getNormal(v1);
            const auto &n2 = mesh.getNormal(v2);
            normal = normalize(intr.i * n0 + intr.j * n1 + intr.k * n2);
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
        properties.uv = intr.i*t0 + intr.j*t1 + intr.k*t2;
        #endif

        properties.normal = normal;
        properties.material = &mesh.getMaterial(mesh.getPrimToMaterial(intr.primId));
    } else if constexpr (std::is_same<U, SoaRayIntersection>::value) {
        __builtin_trap();
    } else {
        __builtin_trap();
    }
}

template void Scene::getSurfaceProperties<SurfaceProperties, RayIntersection>(SurfaceProperties&, const RayIntersection&) const;

template<typename T, typename R>
void Scene::intersect(T& intr, const R& ray) const
{
    memset(&intr, 0, sizeof(intr));

    // Find intersection closer than maxT
    intr.t = ray.maxT;

    // TODO bbox/as for bvhs
    for (auto &bvh : m_bvh) {
        // TODO ray transform per BVH

        bvh->intersect(intr, ray);
    }

    if constexpr (std::is_same<R, Ray>::value) {
        if (intr.t == ray.maxT) {
            intr.t = RayIntersection::kNoHitT;
        }
    } else if constexpr (std::is_same<R, SoaRay>::value) {
        auto mask = intr.t.equals(ray.maxT);
        intr.t = select(intr.t, SoaRayIntersection::kNoHitT, mask);
    }    
}

template void Scene::intersect<RayIntersection, Ray>(RayIntersection&, const Ray&) const;
template void Scene::intersect<SoaRayIntersection, SoaRay>(SoaRayIntersection&, const SoaRay&) const;


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