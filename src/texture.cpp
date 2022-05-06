#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <filesystem>

#define STB_IMAGE_IMPLEMENTATION
#include "../ext/stb/stb_image.h"

#define TINYEXR_IMPLEMENTATION
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-but-set-variable"
#pragma clang diagnostic ignored "-Wunused-function"
#include "../ext/tinyexr/tinyexr.h"
#pragma clang diagnostic pop

#include "prt.h"
#include "log.h"
#include "vecmath.h"
#include "image.h"

#include "material.h"


#define DELETE_ARRAY(x) { delete[] x; x = nullptr; }

namespace prt
{


inline void computeBilinearIndicesAndWeights(float* k, int32_t* indices, int32_t component, const Vector2f& uv, int32_t width, int32_t height)
{
    float s = uv.x - std::floor(uv.x);
    float t = uv.y - std::floor(uv.y);

    int32_t x[2];
    x[0] = std::floor(s*(width - 1));
    x[1] = std::min(x[0] + 1, width - 1);
    int32_t y[2];
    y[0] = std::floor(t*(height - 1));
    y[1] = std::min(y[0] + 1, height - 1);

    indices[0] = component*(x[0] + y[0]*width);
    indices[1] = component*(x[1] + y[0]*width);
    indices[2] = component*(x[0] + y[1]*width);
    indices[3] = component*(x[1] + y[1]*width);

    k[0] = (1.0f - s)*(1.0f - t);
    k[1] = s*(1.0f - t);
    k[2] = (1.0f - s)*t;
    k[3] = s*t;
}

inline void computeBilinearIndicesAndWeights(SoaFloat* k, SoaInt* indices, int32_t component, const SoaVector2f& uv, int32_t width, int32_t height)
{
    auto s = uv.getX() - floor(uv.getX());
    auto t = uv.getY() - floor(uv.getY());

    SoaInt x[2];
    x[0] = floor(s*(width - 1));
    x[1] = min(x[0] + 1, width - 1);
    SoaInt y[2];
    y[0] = floor(t*(height - 1));
    y[1] = min(y[0] + 1, height - 1);

    indices[0] = component*(x[0] + y[0]*width);
    indices[1] = component*(x[1] + y[0]*width);
    indices[2] = component*(x[0] + y[1]*width);
    indices[3] = component*(x[1] + y[1]*width);

    k[0] = (1.0f - s)*(1.0f - t);
    k[1] = s*(1.0f - t);
    k[2] = (1.0f - s)*t;
    k[3] = s*t;
}

template<typename T>
T Texture::sample(const Vector2f& uv) const
{
    if (channel == TextureChannel::k8Unorm) {
        return sample<T, uint8_t>(uv);
    } else if (channel == TextureChannel::k32Float) {
        return sample<T, float>(uv);
    }

    PRT_ASSERT(false);
}

template float Texture::sample<float>(const Vector2f&) const;
template Vector3f Texture::sample<Vector3f>(const Vector2f&) const;
template Vector4f Texture::sample<Vector4f>(const Vector2f&) const;


template<typename T, typename C>
T Texture::sample(const Vector2f& uv) const
{
    float k[4];
    int32_t pindex[4];
    computeBilinearIndicesAndWeights(k, pindex, component, uv, width, height);

    const C* p = (C*)texels;
    T c = 0.0f;
    for (uint32_t i = 0; i < 4; i++) {
        const uint32_t base = pindex[i];

// TODO: respect component
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

    // TODO: may want to define data types such as unorm, snorm, float, ...
    if constexpr(std::is_same<C, uint8_t>::value) {
        const float kChannelScale = 1.0f/255.0f;
        return kChannelScale*c;
    } else if constexpr(std::is_same<C, float>::value) {
        return c;
    }
}


bool Texture::testAlpha(const Vector2f& uv) const
{
    float k[4];
    int32_t pindex[4];
    computeBilinearIndicesAndWeights(k, pindex, component, uv, width, height);

    const uint8_t* p = (uint8_t*)texels;
    float alpha = 0.0f;
    for (uint32_t i = 0; i < 4; i++) {
        uint32_t base = pindex[i];
        alpha = alpha + k[i]*p[base + kAlphaOffset];
    }

    return alpha > kAlphaThreashold;
}

SoaMask Texture::testAlpha(const SoaMask& mask, const SoaVector2f& uv) const
{
    SoaFloat k[4];
    SoaInt pindex[4];
    computeBilinearIndicesAndWeights(k, pindex, component, uv, width, height);

    // do - while can be used here because mask shouldn't be zero
    uint32_t maskAlpha = 0;
    auto bits = mask.ballot();
    do {
        int32_t index = bitScanForward(bits);
        // TODO: vectorize...
        const uint8_t* p = (uint8_t*)texels;
        float alpha = 0.0f;
        for (uint32_t i = 0; i < 4; i++) {
            uint32_t base = pindex[i].getLane(index);
            alpha = alpha + k[i].getLane(index)*p[base + kAlphaOffset];
        }
        if (alpha > kAlphaThreashold)
            maskAlpha |= 1 << index;

        bits &= bits - 1;
    } while(bits);

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

void Texture::init()
{
    width = 0;
    height = 0;
    format = 0;
    component = 0;
    channel = TextureChannel::k8Unorm;
    texels = nullptr;
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
    component = req_comp;
    channel = TextureChannel::k8Unorm;

    if (p) {
        if (bumpTexture && comp == 3) {
            component = 1;
        }

        const uint32_t size = width*height*component; 
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

void Texture::loadExr(const char* path)
{
    int32_t exrWidth;
    int32_t exrHeight;
    const char* err = nullptr;
    int32_t res;

    auto fp = fopen(path, "r");
    if (fp == nullptr) {
        logPrintf(LogLevel::kError, "Failed to open '%s'\n", path);
        return;
    }

    res = fseek(fp, 0, SEEK_END);
    PRT_ASSERT(res == 0);
    auto exrSize = ftell(fp);
    res = fseek(fp, 0, SEEK_SET);
    PRT_ASSERT(res == 0);

    struct TempBuffer {
        uint8_t* buffer;
        TempBuffer(uint32_t size) {
            buffer = (uint8_t*)malloc(size);
        }
        ~TempBuffer() {
            free(buffer);
            buffer = nullptr;
        }
    };

    TempBuffer temp(exrSize);
    auto exrBuffer = temp.buffer;
    res = fread(exrBuffer, exrSize, 1, fp);
    PRT_ASSERT(res == 1);
    fclose(fp);

    float* p;
    res = LoadEXRFromMemory(&p, &exrWidth, &exrHeight, exrBuffer, exrSize, &err);
    if (res != TINYEXR_SUCCESS) {
        if (err) {
            logPrintf(LogLevel::kError, "Loading .exr error : %s\n", err);
            FreeEXRErrorMessage(err);
            return;
        }
    }

    logPrintf(LogLevel::kVerbose, "Loaded .exr '%s' (%d, %d)\n", path, exrWidth, exrHeight);

    texels = p;
    component = 4;

    width = exrWidth;
    height = exrHeight;
    format = 0;
    channel = TextureChannel::k32Float;

#if 0
    float maxInt = 0.0f;
    uint32_t mx = 0;
    uint32_t my = 0;
//    Image wimage(image.width, image.height);
    for (uint32_t y = 0; y < height; y++) {
        for (uint32_t x = 0; x < width; x++) {
            for (uint32_t i = 0; i < 4; i++) {
                uint32_t li = p[4*(x + y*width) + i];
                if (li > maxInt) {
                    maxInt = li;
                    mx = x;
                    my = y;
                }
            }
//            Vector3f c(p[i + 0], p[i + 1], p[i + 2]);
//            c = min(c, Vector3f(1.0f));
//            wimage.WritePixel(x, y, c);
        }
    }
    printf("max int=%f @(%u, %u)\n", maxInt, mx, my);

 //   wimage.SaveToPpm("temp_exr.ppm");
#endif
}

bool Texture::isAlphaTestRequired() const
{
    if (!isValid())
        return false;

    for (int i = 0; i < width*height; ++i) {
        if (((uint8_t*)texels)[i*4 + kAlphaOffset] < 255) {
            return true;
        }
    }

    return false;
}

} // namespace prt