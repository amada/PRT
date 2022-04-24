#pragma once

#include "vecmath.h"

namespace prt
{

enum class TextureChannel : uint8_t {
    k8Unorm = 0,
    k32Float = 1
};

struct Texture
{
    friend struct Material;

    uint16_t width;
    uint16_t height;
    uint16_t format;
    uint8_t component; // components per texel
    TextureChannel channel;
    void* texels;

    void init();

    bool isValid() const { return texels != nullptr; }

    float onePixel() const { return 0.5f/width + 0.5f/height; }

    template<typename T>
    T sample(const Vector2f& uv) const;
    bool testAlpha(const Vector2f& uv) const;
    SoaMask testAlpha(const SoaMask& mask, const SoaVector2f& uv) const;

    bool isAlphaTestRequired() const;
    void load(const char* path, bool bumpTexture = false);
    void loadExr(const char* path);

private:
    template<typename T, typename C>
    T sample(const Vector2f& uv) const;


    const int32_t kAlphaThreashold = 127;
    const int32_t kAlphaOffset = 3;
};


} // namespace prt
