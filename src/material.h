#pragma once

#include <string>

#include "vecmath.h"
#include "texture.h"

namespace tinyobj
{
    struct material_t;
}

namespace prt
{

class Mesh;

struct Material;
template<typename T, typename U>
struct SurfacePropertiesT;
using SurfaceProperties = SurfacePropertiesT<Vector3f, const Material*>;


enum class ReflectionType : uint32_t {
    kDiffuse = 0,
    kSpecular,
    kRefraction
};

struct Material
{
    friend class Mesh;
//    String name;
    Vector3f diffuse;
    Vector3f ambient;
    Vector3f specular;
    Vector3f emissive;
    Texture diffuseMap;
    Texture ambientMap;
    Texture specularMap;
    Texture emissiveMap;
    Texture bumpMap;
    ReflectionType reflectionType;
    bool alphaTest;

    void init();

    // TODO: support for alpha channel?
    Vector3f sampleDiffuse(const Vector2f& uv) const;
    Vector3f sampleBump(const SurfaceProperties& prop) const;
    bool testAlpha(const Vector2f& uv) const {
        return diffuseMap.testAlpha(uv);
    }

    SoaMask testAlpha(const SoaMask& mask, const SoaVector2f& uv) const {
        return diffuseMap.testAlpha(mask, uv);
    }

private:
    void load(const tinyobj::material_t& m, const std::string& dirPath);
};


} // namespace prt
