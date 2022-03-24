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

#if 0
// TODO pod?
class String final
{
public:
    String(const char* str) : m_str(str) {}

private:
    std::string m_str;
};
#endif

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
    uint16_t padding;
    void* texels;

// TODO: Sample texture with other than 4 channels
//    template<typename T>
    bool isValid() const { return texels != nullptr; }
    Vector4f sample(const Vector2f& uv) const;
private:
    void load(const char* path);
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
    Texture normalMap;
    ReflectionType reflectionType;

    // TODO: support for alpha channel?
    Vector3f sampleDiffuse(const Vector2f& uv) const;
private:
    void load(const tinyobj::material_t& m, const std::string& dirPath);
};


} // namespace prt
