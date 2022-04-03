#pragma once

#include "vecmath.h"
#include "bvh.h"
#include "mesh.h"
#include "light.h"

namespace prt
{

template<typename T, typename U>
struct SurfacePropertiesT
{
    T normal;
    U material;
    Vector2f uv;
};

using SurfaceProperties = SurfacePropertiesT<Vector3f, const Material*>;
using SoaSurfaceProperties = SurfacePropertiesT<SoaVector3f, SoaVar<const Material*>>;


class Scene
{
public:
    void init();

    template<typename T, typename R>
    void intersect(T& intr, const R& ray) const;

    template<typename T, typename R>
    T occluded(const R& ray) const;

    template<typename T, typename U>
    void getSurfaceProperties(T& properties, const U& intr) const;

    void add(Bvh* bvh) { 
        bvh->m_id = m_bvh.size();
        m_bvh.push_back(bvh); 
    } // shared ptr?

    void setDirectionalLight(const Vector3f& dir, const Vector3f& intensity) {
        m_directionalLight = {dir, intensity};
    }

    DirectionalLight getDirectionalLight() const {
        return m_directionalLight;
    }

private:
    // TODO remove std::vector
    std::vector<Bvh*> m_bvh;

    // TODO: support for multiple lights
    DirectionalLight m_directionalLight;
};


} // namespace prt
