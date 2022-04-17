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
    T occluded(const R& packet) const;

    template<typename T, typename U>
    void getSurfaceProperties(T& properties, const U& hit) const;

    void add(Bvh* bvh) { 
        bvh->m_mesh.m_id = m_bvh.size();
        m_bvh.push_back(bvh); 
    } // shared ptr?

    void setDirectionalLight(const Vector3f& dir, const Vector3f& intensity) {
        m_directionalLight = {dir, intensity};
    }

    DirectionalLight getDirectionalLight() const {
        return m_directionalLight;
    }

    void createStackCache(TraverseStackCache& stackCache, const Vector3f& pos, const Vector3f& dir) const {
        PRT_ASSERT(m_bvh.size() == 1); // TODO: currently only support one BVH
        m_bvh[0]->createStackCache(stackCache, pos, dir);
    }

private:
    // TODO remove std::vector
    std::vector<Bvh*> m_bvh;

    // TODO: support for multiple lights
    DirectionalLight m_directionalLight;
};


} // namespace prt
