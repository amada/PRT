#pragma once

#include "vecmath.h"
#include "bvh.h"
#include "mesh.h"
#include "light.h"

namespace prt
{

class Scene
{
public:
    void init();

    template<typename T, typename R>
    void intersect(T& intr, const R& packet, const TraverseStackCache* stackCache = nullptr) const;

    template<typename T, typename R>
    T occluded(const RayPacketMask& mask, const R& packet) const;

    template<typename T, typename U>
    void getSurfaceProperties(T& properties, const U& hit) const;

    void add(Bvh* bvh);

    bool isLightAvailable(LightType type) const {
        return (m_availableLights & (1 << (int32_t)type)) != 0;
    }

    void setDirectionalLight(const Vector3f& dir, const Vector3f& intensity) {
        m_directionalLight = {dir, intensity};
        m_availableLights |= (1 << (int32_t)LightType::kDirectional);
        m_availableLights &= ~(1 << (int32_t)LightType::kInfiniteArea);

    }

    const DirectionalLight& getDirectionalLight() const {
        return m_directionalLight;
    }

    void setInfiniteAreaLight(const char* path) {
        m_infiniteAreaLight.create(path);
        m_availableLights |= (1 << (int32_t)LightType::kInfiniteArea);
        m_availableLights &= ~(1 << (int32_t)LightType::kDirectional);
    }

    const InfiniteAreaLight& getInfiniteAreaLight() const {
        return m_infiniteAreaLight;
    }

    void createStackCache(TraverseStackCache& stackCache, const Vector3f& pos, const Vector3f& dir) const {
        PRT_ASSERT(m_bvh.size() == 1); // TODO: currently only support one BVH
        m_bvh[0]->createStackCache(stackCache, pos, dir);
    }

    float getRadius() const {
        return m_radius;
    }

private:
    // TODO remove std::vector
    std::vector<Bvh*> m_bvh;

    // TODO: support for multiple lights
    uint32_t m_availableLights;
    DirectionalLight m_directionalLight;
    InfiniteAreaLight m_infiniteAreaLight;

    BBox m_bbox;
    float m_radius;
};


} // namespace prt
