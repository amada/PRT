#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <filesystem>

#include "log.h"
#include "vecmath.h"
#include "triangle.h"
#include "mesh.h"

#include "../ext/tinyobjloader/tiny_obj_loader.h"
#define STB_IMAGE_IMPLEMENTATION
#include "../ext/stb/stb_image.h"

#include "material.h"


#define DELETE_ARRAY(x) { delete[] x; x = nullptr; }

namespace prt
{

inline Vector3f degamma(const Vector3f& c)
{
    return Vector3f(pow(c.x, 2.2f), pow(c.y, 2.2f), pow(c.z, 2.2f));
}

inline void computeBilinearIndicesAndWeights(float* k, int32_t* indices, int32_t bytesPerPixel, const Vector2f& uv, uint32_t width, uint32_t height)
{
    float s = uv.x - std::floor(uv.x);
    float t = 1.0f - (uv.y - std::floor(uv.y));

    int32_t x[2];
    x[0] = std::floor(s*(width - 1));
    x[1] = std::ceil(s*(width - 1));
    int32_t y[2];
    y[0] = std::floor(t*(height - 1));
    y[1] = std::ceil(t*(height - 1));

    indices[0] = bytesPerPixel*(x[0] + y[0]*width);
    indices[1] = bytesPerPixel*(x[1] + y[0]*width);
    indices[2] = bytesPerPixel*(x[0] + y[1]*width);
    indices[3] = bytesPerPixel*(x[1] + y[1]*width);

//    float k[4];
    k[0] = (1.0f - s)*(1.0f - t);
    k[1] = s*(1.0f - t);
    k[2] = (1.0f - s)*t;
    k[3] = s*t;
}

template<typename T>
T Texture::sample(const Vector2f& uv) const
{
    float k[4];
    int32_t pindex[4];
    computeBilinearIndicesAndWeights(k, pindex, bytesPerPixel, uv, width, height);

    const uint8_t* p = (uint8_t*)texels;
    T c = 0.0f;
    for (uint32_t i = 0; i < 4; i++) {
        const uint32_t base = pindex[i];

// TODO: respect bytesPerPixel
        if constexpr(std::is_same<T, Vector4f>::value) {
            c = c + k[i]*Vector4f(p[base + 0], p[base + 1], p[base + 2], p[base + 3]);
        } else if constexpr(std::is_same<T, Vector3f>::value) {
            c = c + k[i]*Vector3f(p[base + 0], p[base + 1], p[base + 2]);
        } else if constexpr(std::is_same<T, Vector2f>::value) {
            c = c + k[i]*Vector2f(p[base + 0], p[base + 1]);
        } else if constexpr(std::is_same<T, float>::value) {
            c = c + k[i]*p[base + 0];
        } else {
            printf("Unsupported texture type");
            __builtin_trap();
        }
    }

    const float kChannelScale = 1.0f/255.0f;
    return kChannelScale*c;
}

bool Texture::testAlpha(const Vector2f& uv) const
{
    float k[4];
    int32_t pindex[4];
    computeBilinearIndicesAndWeights(k, pindex, bytesPerPixel, uv, width, height);

    const uint8_t* p = (uint8_t*)texels;
    float alpha = 0.0f;
    for (uint32_t i = 0; i < 4; i++) {
        uint32_t base = pindex[i];
        alpha = alpha + k[i]*p[base + 3];
    }

    return alpha > kAlphaThreashold;
}

SoaMask Texture::testAlpha(const SoaMask& mask, const SoaVector2f& uv) const
{
    uint32_t maskAlpha = 0;

    auto s = uv.getX() - uv.getX().floor();
    auto t = 1.0f - (uv.getY() - uv.getY().floor());
    SoaInt x = s*(width - 1.0f);
    SoaInt y = t*(height - 1.0f);
    auto _bindex = 4*(x + y*width);

    // TODO: do ~ while can be used because mask shouldn't be zero
    auto bits = mask.ballot();
    while (bits) {
        int32_t index = bitScanForward(bits);
        auto bindex = _bindex.getLane(index);
        const uint8_t* p = (uint8_t*)texels;

        if (p[bindex + 3] > kAlphaThreashold)
            maskAlpha |= 1 << index;

        bits &= bits - 1;
    }

    return SoaMask(maskAlpha);
}

void convertNormalToBump(uint8_t* bump, const uint8_t* normal, uint32_t width, uint32_t height)
{
    const float kChannelScale = 1.0f/255.0f;
    for (uint32_t y = 0; y < height; y++) {
        for (uint32_t x = 0; x < width; x++) {
            uint32_t index = 4*(x + y*width);
            float nx = 2.0f*(normal[index + 0]*kChannelScale - 0.5f);
            float ny = 2.0f*(normal[index + 1]*kChannelScale - 0.5f);
            float nz = 2.0f*(normal[index + 2]*kChannelScale - 0.5f);
            auto n = normalize(Vector3f(nx, ny, nz));

            uint32_t bindex = x + y*width;
            bump[bindex] = 0xff*clamp(n.z*n.z*n.z*n.z, 0.0f, 1.0f);
        }
    }
}

void Texture::load(const char* path, bool bumpTexture)
{
    int w = 0;
    int h = 0;
    int comp = 0;

    if (!stbi_info(path, &w, &h, &comp)) {
        logPrintf(LogLevel::kError, "Failed stbi_info '%s' %s\n", path, stbi_failure_reason());
        return;
    }

    int req_comp = STBI_rgb_alpha;
    if (comp == 1)
        req_comp = STBI_grey;

    auto p = stbi_load(path, &w, &h, &comp, req_comp);

    width = w;
    height = h;
    format = 0;
    bytesPerPixel = req_comp;

    if (p) {
        if (bumpTexture && comp == 3) {
            bytesPerPixel = 1;
        }

        const uint32_t size = width*height*bytesPerPixel; 
        texels = new uint8_t[size];

        if (bumpTexture && comp == 3) {
            convertNormalToBump((uint8_t*)texels, (uint8_t*)p, width, height);
        } else {
            memcpy(texels, p, size);
        }

        stbi_image_free(p);
        logPrintf(LogLevel::kVerbose, "Loaded '%s' (%d, %d) comp=%d, texels=%p\n", path, w, h, comp, p);
    } else {
        logPrintf(LogLevel::kError, "Failed loading '%s' %s\n", path, stbi_failure_reason());
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