#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <filesystem>

#include "../ext/tinyobjloader/tiny_obj_loader.h"

#include "log.h"
#include "vecmath.h"
#include "triangle.h"
#include "mesh.h"


#include "material.h"


#define DELETE_ARRAY(x) { delete[] x; x = nullptr; }

namespace prt
{


inline Vector3f degamma(const Vector3f& c)
{
    // Possibly square can be used to replace pow. Pre-calculation is needed when loading texture, though
    return Vector3f(pow(c.x, 2.2f), pow(c.y, 2.2f), pow(c.z, 2.2f));
}

void Material::load(const tinyobj::material_t& m, const std::string& dirPath)
{
    diffuse = Vector3f(m.diffuse, std::size(m.diffuse));
    ambient = Vector3f(m.ambient, std::size(m.ambient));
    specular = Vector3f(m.specular, std::size(m.specular));
    emissive = Vector3f(m.emission, std::size(m.emission));

    auto basePath = dirPath + static_cast<char>(std::filesystem::path::preferred_separator);

    auto replaceSeparator = [](const auto& path) {
        std::string result;
        for (auto c : path) {
            if (c == '\\') {
                result += '/';
            } else {
                result += c;
            }
        }
        return result;
    };

    if (m.diffuse_texname.size() > 0)
        diffuseMap.load(replaceSeparator(basePath + m.diffuse_texname).c_str());
    if (m.ambient_texname.size() > 0)
        ambientMap.load(replaceSeparator(basePath + m.ambient_texname).c_str());
    if (m.specular_texname.size() > 0)
        specularMap.load(replaceSeparator(basePath + m.specular_texname).c_str());
    if (m.emissive_texname.size() > 0)
        emissiveMap.load(replaceSeparator(basePath + m.emissive_texname).c_str());
    if (m.bump_texname.size() > 0)
        bumpMap.load(replaceSeparator(basePath + m.bump_texname).c_str(), true);

    alphaTest = diffuseMap.isAlphaTestRequired();
#if 0
    if (alphaTest) {
        logPrintf(LogLevel::kVerbose, "Alpha test required for '%s'\n", m.name.c_str());
    } else {
        logPrintf(LogLevel::kVerbose, "Alpha test not required for '%s'\n", m.name.c_str());
    }
#endif
}

Vector3f Material::sampleDiffuse(const Vector2f& uv) const
{
    auto color = diffuse;
    auto& tex = diffuseMap;
    if (tex.isValid()) {
        color = color*degamma(tex.sample<Vector3f>(uv));
    }

    return color;
}

Vector3f Material::sampleBump(const SurfaceProperties& prop) const
{
    auto normal = prop.normal;

    auto& tex = bumpMap;
    if (tex.isValid()) {
        auto uv = prop.uv;
        auto b = tex.sample<float>(uv);
        auto b01 = tex.sample<float>(uv + tex.onePixel()*prop.duv01) - b;
        auto b02 = tex.sample<float>(uv + tex.onePixel()*prop.duv02) - b;

        float nk = 4.0f;
        normal = normalize(normal + nk*b01*prop.dp01 + nk*b02*prop.dp02);
    }

    return normal;
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