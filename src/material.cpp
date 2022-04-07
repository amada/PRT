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

bool Texture::testAlpha(const Vector2f& uv) const
{
    float s = uv.x - std::floor(uv.x);
    float t = 1.0f - (uv.y - std::floor(uv.y));
    int32_t x = s*(width - 1);
    int32_t y = t*(height - 1);
    int32_t bindex = 4*(x + y*width);
    uint8_t* p = (uint8_t*)texels;

    return p[bindex + 3] > kAlphaThreashold;
}

SoaMask Texture::testAlpha(const SoaMask& mask, const SoaVector2f& uv) const
{
    uint32_t maskAlpha = 0;

    auto s = uv.getX() - uv.getX().floor();
    auto t = 1.0f - (uv.getY() - uv.getY().floor());
    SoaInt x = s*(width - 1.0f);
    SoaInt y = t*(height - 1.0f);
    auto _bindex = 4*(x + y*width);

    auto bits = mask.ballot();
    while (bits) {
        int32_t index = bitScanForward(bits);
        auto bindex = _bindex.getLane(index);
        uint8_t* p = (uint8_t*)texels;

        if (p[bindex + 3] > kAlphaThreashold)
            maskAlpha |= 1 << index;

        bits &= bits - 1;
    }

    return SoaMask(maskAlpha);
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

    if (p == nullptr) {
        logPrintf(LogLevel::kError, "Failed loading '%s' %s\n", path, stbi_failure_reason());
    } else {
        logPrintf(LogLevel::kVerbose, "Loaded '%s' (%d, %d) %p\n", path, w, h, p);
    }
}

bool Texture::isAlphaTestRequired() const
{
    if (!isValid())
        return false;

    for (int i = 0; i < width*height; ++i) {
        if (((uint8_t*)texels)[i*4 + 3] < 255) {
            return true;
        }
    }

    return false;
}

void Material::load(const tinyobj::material_t& m, const std::string& dirPath)
{
    diffuse = Vector3f(m.diffuse, std::size(m.diffuse));
    ambient = Vector3f(m.ambient, std::size(m.ambient));
    specular = Vector3f(m.specular, std::size(m.specular));
    emissive = Vector3f(m.emission, std::size(m.emission));

    auto basePath = dirPath + std::filesystem::path::preferred_separator;

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
        normalMap.load(replaceSeparator(basePath + m.bump_texname).c_str());

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