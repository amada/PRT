#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <filesystem>
//#include <fstream>

#include "log.h"
//#include "string_util.h"
#include "vecmath.h"
#include "triangle.h"

#include "../ext/tinyobjloader/tiny_obj_loader.h"
#define STB_IMAGE_IMPLEMENTATION
#include "../ext/stb/stb_image.h"

#include "material.h"


#define DELETE_ARRAY(x) { delete[] x; x = nullptr; }

namespace prt
{

Vector4f Texture::sample(const Vector2f& uv) const
{
    const float kChannelScale = 1.0f/255.0f;
    float s = uv.x - std::floor(uv.x);
    float t = 1.0f - (uv.y - std::floor(uv.y));
    int32_t x = s*(width - 1);
    int32_t y = t*(height - 1);
    int32_t bindex = 4*(x + y*width);
    uint8_t* p = (uint8_t*)texels;

    return Vector4f(p[bindex + 0]*kChannelScale, p[bindex + 1]*kChannelScale, p[bindex + 2]*kChannelScale, p[bindex + 3]*kChannelScale);
}

void Texture::load(const char* path)
{
    int w;
    int h;
    int comp;
    auto p = stbi_load(path, &w, &h, &comp, STBI_rgb_alpha);

    width = w;
    height = h;
    format = 0;
    texels = p;

    logPrintf(LogLevel::kVerbose, "load '%s' (%d, %d) %p\n", path, w, h, p);
}

void Material::load(const tinyobj::material_t& m, const std::string& dirPath)
{
    diffuse = Vector3f(m.diffuse, std::size(m.diffuse));
    ambient = Vector3f(m.ambient, std::size(m.ambient));
    specular = Vector3f(m.specular, std::size(m.specular));
    emissive = Vector3f(m.emission, std::size(m.emission));

    auto basePath = dirPath + std::filesystem::path::preferred_separator;

    diffuseMap.load((basePath + m.diffuse_texname).c_str());
    ambientMap.load((basePath + m.ambient_texname).c_str());
    specularMap.load((basePath + m.specular_texname).c_str());
    emissiveMap.load((basePath + m.emissive_texname).c_str());
    normalMap.load((basePath + m.bump_texname).c_str());
}

Vector3f Material::sampleDiffuse(const Vector2f& uv) const
{
    auto color = diffuse;
    auto& tex = diffuseMap;
    if (tex.isValid()) {
        color = color*tex.sample(uv).getXYZ();
    }

    return color;
}

#if 0
MaterialLibrary* MaterialLibrary::s_instance;

void MaterialLibrary::loadMtl(const char* path)
{
    std::ifstream file(path);

    if (!file.is_open()) {
        logPrintf(LogLevel::kError, "Failed to open '%s'\n", path);
        return;
    }

    logPrintf(LogLevel::kVerbose, "Loading '%s'\n", path);

    std::string line;
    while (getline(file, line)) {
        std::string::size_type pos = 0;
//        auto str = findToken(line, pos, ' ');
        auto str = findTokenWhitespace(line, pos);
//        printf("%s;\n", str.c_str());

        if (str == "newmtl") {
            auto name = findTokenWhitespace(line, pos);
            printf("newmtl %s;\n", name.c_str());
        } else if (str == "Ns") {
        }

    }

    file.close();
}

void MaterialLibrary::loadTexture(const char* path)
{
    std::ifstream file(path);

    file.close();
}
#endif

} // namespace prt