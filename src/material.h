#pragma once

#include <string>

#include "vecmath.h"

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


struct Texture
{
    friend struct Material;

    uint16_t width;
    uint16_t height;
    uint16_t format;
    uint16_t bytesPerPixel;
    void* texels;

// TODO: Sample texture with other than 4 channels
//    template<typename T>
    bool isValid() const { return texels != nullptr; }

    float onePixel() const { return 0.5f/width + 0.5f/height; }

    template<typename T>
    T sample(const Vector2f& uv) const;
    bool testAlpha(const Vector2f& uv) const;
    SoaMask testAlpha(const SoaMask& mask, const SoaVector2f& uv) const;

    bool isAlphaTestRequired() const;
private:
    const int32_t kAlphaThreashold = 127;

    void load(const char* path, bool bumpTexture = false);
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
