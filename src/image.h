#pragma once

#include "vecmath.h"

namespace prt
{

class Image
{
public:
    Image(uint32_t width, uint32_t height, bool tonemap = true, float exposure = 1.0f);
    virtual ~Image();

    void writePixel(uint32_t x, uint32_t y, const Vector3f& color);
    void saveToPpm(const char* path) const;

    uint32_t getWidth() const { return m_width; }
    uint32_t getHeigit() const { return m_height; }
private:
    const static uint32_t kChannelsPerPixel = 3;
    float gamma(float f)
    {
        return pow(f, 1/2.2f);
    }

    uint8_t* m_pixels;
    uint32_t m_width;
    uint32_t m_height;
    bool m_tonemap;
    float m_exposure;
};

} // namespace prt
